#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import time
import math
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

class CalibrationVerification(Node):
    def __init__(self):
        super().__init__('calibration_verification')
        
        # Parameters
        self.target_frame = 'base_link' 
        self.camera_frame = 'camera_color_optical_frame'
        self.tool_link = 'tool0'
        
        # Callback Group for MultiThreadedExecutor
        self.callback_group = ReentrantCallbackGroup()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 1, callback_group=self.callback_group)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 1, callback_group=self.callback_group)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 1, callback_group=self.callback_group)
        
        # Publishers
        self.debug_pub = self.create_publisher(Image, '/calibration_verification/debug_image', 1, callback_group=self.callback_group)
        
        # State
        self.latest_color = None
        self.latest_depth = None
        self.camera_intrinsics = None
        self.frame_count = 0
        self.last_update_time = 0
        self.initial_fixed_x = None # Store the initial X position
        self.is_moving = False # Flag to track robot movement status
        self.position_history = [] # For stability check
        self.stability_threshold = 0.01 # 1cm stability threshold
        
        # UR3e
        self.ur_ip = "192.168.1.10"
        self.rtde_c = RTDEControlInterface(self.ur_ip)
        self.rtde_r = RTDEReceiveInterface(self.ur_ip)
        self.get_logger().info("✓ 已连接到 UR3e")
        
        # Tracking Mode Selection
        self.tracking_mode = 'tcp' # Default to TCP center tracking
        self._select_tracking_mode()
        
        self.get_logger().info(f'=== 木块实时跟踪节点已启动 (模式: {self.tracking_mode.upper()}) ===')
        self.get_logger().info('等待相机数据...')

    def _select_tracking_mode(self):
        """Ask user for tracking mode"""
        print("\n" + "="*40)
        print("请选择跟踪模式 / Select Tracking Mode:")
        print("1. 相机中心对准 (Camera Center Alignment)")
        print("   - 保持木块在图像中心")
        print("   - Keep object in center of image")
        print("2. 工具中心对准 (TCP Center Alignment)")
        print("   - 保持夹爪中心对准木块 (抓取预备状态)")
        print("   - Keep TCP/Gripper center aligned with object")
        print("="*40)
        
        while True:
            try:
                choice = input("输入选项 (1/2): ").strip()
                if choice == '1':
                    self.tracking_mode = 'camera'
                    break
                elif choice == '2':
                    self.tracking_mode = 'tcp'
                    break
                else:
                    print("无效输入，请输入 1 或 2")
            except ValueError:
                pass
        print(f"已选择模式: {self.tracking_mode.upper()}\n")

    def info_callback(self, msg):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = msg
            self.get_logger().info('✓ 相机内参已接收')

    def depth_callback(self, msg):
        self.latest_depth = msg

    def color_callback(self, msg):
        self.latest_color = msg
        self.process_frame()


    def process_frame(self):
        self.frame_count += 1
        
        # Check if robot is moving
        if self.is_moving:
            # Check actual speed to determine if movement finished
            try:
                tcp_speed = self.rtde_r.getActualTCPSpeed()
                speed_mag = np.linalg.norm(tcp_speed[:3])
                
                if speed_mag < 0.001:
                    # Robot stopped
                    current_time = time.time()
                    if current_time - self.last_update_time > 3.0:
                        self.is_moving = False
                        self.position_history = [] # Reset history after move
                        self.get_logger().info("Movement finished. Waiting 3s done. Resuming detection...")
                    else:
                        if self.frame_count % 30 == 0:
                            self.get_logger().info(f"Movement finished. Waiting... ({3.0 - (current_time - self.last_update_time):.1f}s left)")
                        return
                else:
                    if self.frame_count % 30 == 0:
                        self.get_logger().info(f"Robot is moving... Speed: {speed_mag:.4f} m/s")
                    return
            except Exception as e:
                self.get_logger().warn(f"Failed to check robot speed: {e}")
                return

        # 检查依赖数据
        if self.latest_color is None or self.latest_depth is None or self.camera_intrinsics is None:
            if self.frame_count % 30 == 0:
                self.get_logger().warn('等待相机数据...')
            return

        # Convert images
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_color, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth, "passthrough")
            
            # Resize depth to match color if needed (如果相机配置正确，通常不需要)
            if cv_image.shape[:2] != cv_depth.shape[:2]:
                cv_depth = cv2.resize(cv_depth, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)
                
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        debug_img = cv_image.copy()
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)



        # --- Detect Green Object (Wood Block) ---
        lower_green = np.array([40, 80, 80]) 
        upper_green = np.array([95, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_found = False
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            if area > 100: 
                target_found = True
                # Draw largest contour
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(debug_img, (x + w//2, y + h//2), 5, (255, 0, 0), -1)
                
                # Draw rotated rectangle
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(debug_img, [box], 0, (0, 0, 255), 2)
                
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # --- Robust Depth Estimation ---
                    mask_c = np.zeros_like(cv_depth, dtype=np.uint8)
                    cv2.drawContours(mask_c, [c], -1, 255, -1)
                    valid_depths = cv_depth[mask_c > 0]
                    valid_depths = valid_depths[valid_depths > 0]
                    
                    depth_val = 0
                    if len(valid_depths) > 10:
                        depth_val = np.percentile(valid_depths, 5)
                    else:
                        depth_val = cv_depth[cy, cx]
                    
                    if depth_val > 0:
                        z_meters = depth_val * 0.001
                        
                        # Deproject to 3D (Camera Frame)
                        fx = self.camera_intrinsics.k[0]
                        fy = self.camera_intrinsics.k[4]
                        ppx = self.camera_intrinsics.k[2]
                        ppy = self.camera_intrinsics.k[5]
                        
                        x_cam = (cx - ppx) * z_meters / fx
                        y_cam = (cy - ppy) * z_meters / fy  
                        z_cam = z_meters
                        
                        # Transform to Base Link
                        point_cam = tf2_geometry_msgs.PointStamped()
                        point_cam.header.frame_id = self.camera_frame
                        point_cam.header.stamp = rclpy.time.Time().to_msg()
                        point_cam.point.x = x_cam
                        point_cam.point.y = y_cam
                        point_cam.point.z = z_cam
                        
                        try:
                            point_base = self.tf_buffer.transform(point_cam, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1))
                            
                            # --- 6D Orientation Calculation (Side-View Tracking) ---
                            # We want:
                            # 1. Tool Z-axis pointing to Base -X (Forward/Backward depending on definition)
                            # 2. Tool X-axis aligned with object orientation in Y-Z plane
                            # 3. TCP X position fixed (no depth tracking)
                            # 4. TCP Y, Z tracking object
                            
                            # 1. Get Object Angle in Image
                            rect = cv2.minAreaRect(c)
                            box = cv2.boxPoints(rect)
                            box = np.int0(box)
                            
                            p0 = box[0]
                            p1 = box[1]
                            p2 = box[2]
                            d1 = np.linalg.norm(p0-p1)
                            d2 = np.linalg.norm(p1-p2)
                            
                            if d1 > d2:
                                vec_img = p1 - p0
                            else:
                                vec_img = p2 - p1
                                
                            angle_img = math.atan2(vec_img[1], vec_img[0])
                            
                            # 2. Calculate Angle in Base Frame (Y-Z Plane)
                            # We need to transform the vector from Camera to Base
                            # Construct two points in Camera frame
                            # P1 is center (already transformed as point_base)
                            # P2 is a point along the angle
                            
                            cx2 = cx + math.cos(angle_img) * 20
                            cy2 = cy + math.sin(angle_img) * 20
                            
                            x2_cam = (cx2 - ppx) * z_meters / fx
                            y2_cam = (cy2 - ppy) * z_meters / fy
                            z2_cam = z_meters
                            
                            point_cam2 = tf2_geometry_msgs.PointStamped()
                            point_cam2.header.frame_id = self.camera_frame
                            point_cam2.header.stamp = point_cam.header.stamp
                            point_cam2.point.x = x2_cam
                            point_cam2.point.y = y2_cam
                            point_cam2.point.z = z2_cam
                            
                            point_base2 = self.tf_buffer.transform(point_cam2, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1))
                            
                            # Vector in Base Frame
                            vy_base = point_base2.point.x - point_base.point.x # Not used for angle in Y-Z plane
                            vy_base = point_base2.point.y - point_base.point.y
                            vz_base = point_base2.point.z - point_base.point.z
                            
                            # Angle in Y-Z plane
                            angle_yz = math.atan2(vz_base, vy_base)
                            
                            # 3. Construct Target Rotation
                            # Tool Z = [-1, 0, 0] (Base -X)
                            # Tool X should be perpendicular to Z and aligned with object short axis
                            # Object long axis is angle_yz. Short axis is angle_yz + 90.
                            
                            theta = angle_yz + math.pi/2
                            c_t = math.cos(theta)
                            s_t = math.sin(theta)
                            
                            # R = [X_col, Y_col, Z_col]
                            # Z_col = [-1, 0, 0]
                            # X_col = [0, cos(theta), sin(theta)] (In Y-Z plane)
                            # Y_col = Z x X = [0, sin(theta), -cos(theta)]
                            
                            R_target = np.array([
                                [0,   0,   -1],
                                [c_t, s_t,  0],
                                [s_t, -c_t, 0]
                            ])
                            
                            rvec, _ = cv2.Rodrigues(R_target)
                            rx, ry, rz = rvec.flatten()

                            # 4. Calculate Target Pose
                            # Strategy: 
                            # Position: Move in Tool X/Y plane.
                            #           Keep Tool Z (depth) unchanged relative to the object.
                            # Orientation: Keep Current Orientation.
                            
                            # A. Position Calculation
                            # 1. Get Current TCP Pose (Controller Frame)
                            current_tcp_pose = self.rtde_r.getActualTCPPose()
                            current_pos = np.array(current_tcp_pose[:3])
                            current_rot_vec = np.array(current_tcp_pose[3:])
                            R_curr, _ = cv2.Rodrigues(current_rot_vec)
                            
                            offset_ctrl = np.zeros(3)
                            
                            if self.tracking_mode == 'camera':
                                # --- Mode 1: Camera Center Alignment ---
                                # We want to move the tool such that the object (currently at x_cam, y_cam) moves to (0,0) in Camera Frame.
                                # So we shift the tool by (x_cam, y_cam, 0) in Tool Frame.
                                # Note: This assumes Camera Frame axes are aligned with Tool Frame axes (which is roughly true for Eye-in-Hand)
                                
                                offset_tool = np.array([x_cam, y_cam, 0.0])
                                offset_ctrl = R_curr @ offset_tool
                                
                            else: # self.tracking_mode == 'tcp'
                                # --- Mode 2: TCP Center Alignment ---
                                # We want the TCP to move such that the object is at (0,0) in TCP Frame.
                                
                                # Construct Homogeneous Transform Matrix for TCP (T_ctrl_tcp)
                                T_ctrl_tcp = np.eye(4)
                                T_ctrl_tcp[:3, :3] = R_curr
                                T_ctrl_tcp[:3, 3] = current_pos
                                
                                # Get Object Position in Controller Frame
                                # Mapping: X_ctrl = -X_ros, Y_ctrl = -Y_ros, Z_ctrl = Z_ros
                                P_obj_ctrl = np.array([
                                    -point_base.point.x,
                                    -point_base.point.y,
                                    point_base.point.z,
                                    1.0
                                ])
                                
                                # Transform Object to TCP Frame
                                # P_obj_tcp = inv(T_ctrl_tcp) * P_obj_ctrl
                                T_tcp_ctrl = np.linalg.inv(T_ctrl_tcp)
                                P_obj_tcp = T_tcp_ctrl @ P_obj_ctrl
                                
                                # Calculate Offset in TCP Frame
                                # Required Move in TCP: (x, y, 0)
                                offset_tcp = np.array([P_obj_tcp[0], P_obj_tcp[1], 0.0])
                                
                                # Transform Offset to Controller Frame
                                offset_ctrl = R_curr @ offset_tcp
                            
                            # 6. New Target Position
                            target_pos_ctrl = current_pos + offset_ctrl
                            
                            # B. Orientation Calculation (Keep Current Orientation)
                            target_pose = [
                                target_pos_ctrl[0],
                                target_pos_ctrl[1],
                                target_pos_ctrl[2],
                                current_rot_vec[0], 
                                current_rot_vec[1], 
                                current_rot_vec[2],
                            ]
                            
                            # Check reachability (UR3e max reach ~0.5m + Tool Length)
                            dist_from_base = math.sqrt(target_pose[0]**2 + target_pose[1]**2 + target_pose[2]**2)
                            
                            # Stability Check
                            self.position_history.append(target_pose[:3])
                            if len(self.position_history) > 10:
                                self.position_history.pop(0)
                            
                            is_stable = False
                            if len(self.position_history) >= 10:
                                xs = [p[0] for p in self.position_history]
                                ys = [p[1] for p in self.position_history]
                                zs = [p[2] for p in self.position_history]
                                max_dev = max(max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs))
                                
                                if max_dev < self.stability_threshold:
                                    is_stable = True
                                else:
                                    if self.frame_count % 30 == 0:
                                        self.get_logger().info(f"Waiting for stability... Dev: {max_dev:.4f}m")

                            if is_stable:
                                # Move to target (Slow speed)
                                # asynchronous=True to avoid blocking the camera callback
                                self.rtde_c.moveL(target_pose, 0.05, 0.05, asynchronous=True)
                                self.last_update_time = time.time()
                                self.is_moving = True
                                self.get_logger().info(f"Executing move... Target: {target_pose} (Dist: {dist_from_base:.2f}m)")
                            
                            # Logging
                            if self.frame_count % 30 == 0 and not is_stable:
                                try:
                                    tool0_pos = None # Not querying to save time/complexity if not needed
                                    self._log_detection_details(cv_depth.shape, cx, cy, ppx, ppy, z_meters, fx, fy, x_cam, y_cam, z_cam, tool0_pos, None, point_base)
                                    self.get_logger().info(f"Angle Y-Z: {math.degrees(angle_yz):.1f} deg")
                                    self.get_logger().info(f"Mode: {self.tracking_mode} | Target Pose (Controller): {target_pose}")
                                except Exception as e:
                                    self.get_logger().warn(f"Logging error: {e}")

                        except Exception as e:
                            self.get_logger().error(f'Transform/Control Error: {e}')
        
        if not target_found:
            # Stop if lost tracking? Or just stay?
            # self.rtde_c.servoStop()
            pass

        self._publish_debug_image(debug_img)
    
    def _log_detection_details(self, cv_depth_shape, cx, cy, ppx, ppy, z_meters, fx, fy, x_cam, y_cam, z_cam, tool0_pos, tool0_rot, point_base):
        """打印详细的检测和坐标转换信息"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("【坐标转换详细信息】")
        self.get_logger().info(f"1. 图像尺寸: {cv_depth_shape[1]}x{cv_depth_shape[0]} 像素")
        self.get_logger().info(f"2. 检测到的图像坐标: cx={cx}, cy={cy}")
        self.get_logger().info(f"3. 相机光学中心: ppx={ppx:.1f}, ppy={ppy:.1f}")
        self.get_logger().info(f"4. 像素偏移量: dx={cx-ppx:.1f}, dy={cy-ppy:.1f}")
        self.get_logger().info(f"5. 深度值: z={z_meters:.4f}m")
        self.get_logger().info(f"6. 相机内参: fx={fx:.1f}, fy={fy:.1f}")
        self.get_logger().info(f"7. 相机坐标系 (camera_color_optical_frame):")
        self.get_logger().info(f"   X={x_cam:.4f}m, Y={y_cam:.4f}m, Z={z_cam:.4f}m")
        
        if tool0_pos is not None and tool0_rot is not None:
            self.get_logger().info("=" * 60)
            self.get_logger().info("【TF变换链信息】")
            self.get_logger().info(f"8. 当前 tool0 位置 (相对 base_link):")
            self.get_logger().info(f"   X={tool0_pos.x:.4f}m, Y={tool0_pos.y:.4f}m, Z={tool0_pos.z:.4f}m")
            self.get_logger().info(f"   四元数: [{tool0_rot.x:.4f}, {tool0_rot.y:.4f}, {tool0_rot.z:.4f}, {tool0_rot.w:.4f}]")
        
        if point_base is not None:
            self.get_logger().info('='*60)
            self.get_logger().info(f'🎯 检测到木块!')
            self.get_logger().info(f"10. base_link 坐标系:")
            self.get_logger().info(f'    X={point_base.point.x:.4f}m, Y={point_base.point.y:.4f}m, Z={point_base.point.z:.4f}m')
            self.get_logger().info('='*60)

    def _publish_debug_image(self, cv_image):
        """发布调试图像"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'发布调试图像失败: {e}')

    def destroy_node(self):
        # 确保脚本停止，释放机械臂控制权
        try:
            if hasattr(self, 'rtde_c'):
                self.rtde_c.servoStop()
                self.rtde_c.stopScript()
        except Exception as e:
            self.get_logger().warn(f"释放机械臂控制权失败: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationVerification()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
