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
        
        # UR3e
        self.ur_ip = "192.168.1.10"
        self.rtde_c = RTDEControlInterface(self.ur_ip)
        self.rtde_r = RTDEReceiveInterface(self.ur_ip)
        self.get_logger().info("✓ 已连接到 UR3e")
        
        self.get_logger().info('=== 木块实时跟踪节点已启动 ===')
        self.get_logger().info('等待相机数据...')

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
                            
                            # --- 6D Orientation Calculation (Relative to Current Tool Pose) ---
                            # Get Current Tool Pose
                            trans_tool = self.tf_buffer.lookup_transform(self.target_frame, self.tool_link, rclpy.time.Time())
                            q_tool = [trans_tool.transform.rotation.x, trans_tool.transform.rotation.y, trans_tool.transform.rotation.z, trans_tool.transform.rotation.w]
                            import scipy.spatial.transform as st
                            r_tool = st.Rotation.from_quat(q_tool)
                            R_tool_matrix = r_tool.as_matrix()
                            
                            # Calculate Angle in Image (Rotation around Camera Z-axis)
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
                            
                            # We want to align the Tool's X/Y axes with the block's orientation in the image.
                            # The image rotation is around the Camera Z-axis (which is aligned with Tool Z-axis).
                            # So we rotate the Current Tool Orientation by `angle_img` around its local Z-axis.
                            
                            # Create rotation object for the image angle (around Z)
                            r_img = st.Rotation.from_euler('z', angle_img + math.pi/2) # +90 to align X with short axis or similar
                            
                            # New Target Rotation = Current Tool Rotation * Image Rotation
                            # (Note: This assumes we want to KEEP the current approach angle and just spin to match)
                            r_target = r_tool * r_img
                            R_target = r_target.as_matrix()
                            
                            # Convert back to axis-angle for servoL
                            rvec_target = r_target.as_rotvec()
                            rx, ry, rz = rvec_target

                            # Calculate Target Pose (Distance 0.2m along Tool Z-axis)
                            # We want the Tool Center Point (TCP) to be 0.2m away from the object.
                            # The direction from TCP to Object should be the Tool Z-axis (positive depth).
                            # So: P_obj = P_target + (distance * Z_tool)
                            # Therefore: P_target = P_obj - (distance * Z_tool)
                            
                            # Use the CURRENT Tool Z-axis for the offset direction to maintain the current approach vector
                            # (Or use the Target Z-axis, they should be the same if we only rotated around Z)
                            tool_z_axis = R_tool_matrix[:, 2] 
                            distance = 0.2
                            
                            offset_vector = distance * tool_z_axis
                            
                            target_pose = [
                                point_base.point.x - offset_vector[0],
                                point_base.point.y - offset_vector[1],
                                point_base.point.z - offset_vector[2],
                                rx, ry, rz,
                            ]
                            
                            # Check reachability (UR3e max reach ~0.5m)
                            dist_from_base = math.sqrt(target_pose[0]**2 + target_pose[1]**2 + target_pose[2]**2)
                            
                            if dist_from_base > 0.48: # Safety margin for UR3e
                                if self.frame_count % 30 == 0:
                                    self.get_logger().warn(f"Target out of reach! Dist: {dist_from_base:.2f}m (Max ~0.5m)")
                            else:
                                # Servo to target
                                self.rtde_c.servoL(target_pose, 0.0, 0.0, 0.1, 0.1, 300)
                            
                            if self.frame_count % 30 == 0:
                                try:
                                    point_tool = self.tf_buffer.transform(point_cam, self.tool_link, timeout=rclpy.duration.Duration(seconds=0.1))
                                    self.get_logger().info(f"Raw Depth: {z_meters:.4f} m")
                                    self.get_logger().info(f"Pos (Camera): [{x_cam:.4f}, {y_cam:.4f}, {z_cam:.4f}]")
                                    self.get_logger().info(f"Pos (Base): [{point_base.point.x:.4f}, {point_base.point.y:.4f}, {point_base.point.z:.4f}]")
                                    self.get_logger().info(f"Tool Z-Axis: {tool_z_axis}")
                                    self.get_logger().info(f"Offset Applied: {offset_vector}")
                                    self.get_logger().info(f"Target (Base): {target_pose}")
                                    self.get_logger().info(f"Target Dist: {dist_from_base:.2f}m")
                                except Exception as e:
                                    self.get_logger().warn(f"Logging transform error: {e}")

                        except Exception as e:
                            self.get_logger().error(f'Transform/Control Error: {e}')
        
        if not target_found:
            # Stop if lost tracking? Or just stay?
            # self.rtde_c.servoStop()
            pass

        self._publish_debug_image(debug_img)
    
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
