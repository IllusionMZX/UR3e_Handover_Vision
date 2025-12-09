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
from robotiq_2f_urcap_adapter.action import GripperCommand
from robotiq_2f_urcap_adapter.msg import GripperCommand as GripperCommandMsg

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
        
        # Action Client
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_2f_urcap_adapter/gripper_command', callback_group=self.callback_group)

        # State
        self.latest_color = None
        self.latest_depth = None
        self.camera_intrinsics = None
        self.frame_count = 0
        self.is_busy = False
        
        # UR3e
        self.ur_ip = "192.168.1.10"
        self.rtde_c = RTDEControlInterface(self.ur_ip)
        self.rtde_r = RTDEReceiveInterface(self.ur_ip)
        self.get_logger().info("✓ 已连接到 UR3e")
        
        self.position_history = [] 
        self.stability_duration = 10.0 
        self.stability_threshold = 0.01 
        
        self.get_logger().info('=== 木块检测节点已启动 ===')
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

    def send_gripper_command(self, position, speed=0.15, force=140.0):
        """发送夹爪控制命令"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = force
        goal_msg.command.max_speed = speed

        self.get_logger().info(f'Waiting for action server... (Target: {position})')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        self.get_logger().info(f'Sending goal: pos={position}, speed={speed}, force={force}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        while not send_goal_future.done():
            time.sleep(0.1)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        
        # Wait for result
        while not get_result_future.done():
            time.sleep(0.1)
            
        result = get_result_future.result().result
        self.get_logger().info(f'Gripper result: reached_goal={result.reached_goal}, position={result.position}')
        return True

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

        # --- 1. Detect Blue Object (Target) ---
        white_target_point = None
        white_target_angle = 0.0
        # Blue HSV range
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask_white = cv2.inRange(hsv, lower_blue, upper_blue)
        
        kernel = np.ones((5,5),np.uint8)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)
        
        contours_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours_white:
            c_white = max(contours_white, key=cv2.contourArea)
            if cv2.contourArea(c_white) > 500:
                x, y, w, h = cv2.boundingRect(c_white)
                cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(debug_img, "Target", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                
                # --- Calculate Orientation for Blue Block ---
                rect_white = cv2.minAreaRect(c_white)
                box_white = cv2.boxPoints(rect_white)
                box_white = np.int0(box_white)
                
                # Find long edge
                p0_w = box_white[0]
                p1_w = box_white[1]
                p2_w = box_white[2]
                d1_w = np.linalg.norm(p0_w-p1_w)
                d2_w = np.linalg.norm(p1_w-p2_w)
                
                if d1_w > d2_w:
                    vec_img_white = p1_w - p0_w
                else:
                    vec_img_white = p2_w - p1_w
                    
                angle_img_white = math.atan2(vec_img_white[1], vec_img_white[0])

                M = cv2.moments(c_white)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # --- Robust Depth Estimation for Taller Objects ---
                    # Instead of single point depth, use the minimum depth (closest to camera) 
                    # in the contour to ensure we get the top surface height, not the side.
                    mask_c = np.zeros_like(cv_depth, dtype=np.uint8)
                    cv2.drawContours(mask_c, [c_white], -1, 255, -1)
                    
                    # Get all depth values in the contour
                    valid_depths = cv_depth[mask_c > 0]
                    valid_depths = valid_depths[valid_depths > 0] # Remove 0s
                    
                    if len(valid_depths) > 10:
                        # Use 5th percentile to be robust against noise (single flying pixels)
                        # This represents the "top surface" distance
                        depth_val = np.percentile(valid_depths, 5)
                    else:
                        # Fallback to centroid if not enough points
                        depth_val = cv_depth[cy, cx]
                        if depth_val == 0:
                             for dx in range(-5, 6):
                                for dy in range(-5, 6):
                                    d = cv_depth[min(max(cy+dy, 0), cv_depth.shape[0]-1), min(max(cx+dx, 0), cv_depth.shape[1]-1)]
                                    if d > 0:
                                        depth_val = d
                                        break
                                if depth_val > 0: break
                    
                    if depth_val > 0:
                        z_meters = depth_val * 0.001
                        fx = self.camera_intrinsics.k[0]
                        fy = self.camera_intrinsics.k[4]
                        ppx = self.camera_intrinsics.k[2]
                        ppy = self.camera_intrinsics.k[5]
                        
                        x_cam = (cx - ppx) * z_meters / fx
                        y_cam = (cy - ppy) * z_meters / fy  
                        z_cam = z_meters
                        
                        point_cam = tf2_geometry_msgs.PointStamped()
                        point_cam.header.frame_id = self.camera_frame
                        point_cam.header.stamp = rclpy.time.Time().to_msg()
                        point_cam.point.x = x_cam
                        point_cam.point.y = y_cam
                        point_cam.point.z = z_cam
                        
                        # Transform Orientation Vector to Base Frame
                        # Create a second point in image along the angle
                        cx2_white = cx + math.cos(angle_img_white) * 20
                        cy2_white = cy + math.sin(angle_img_white) * 20
                        
                        x2_cam_white = (cx2_white - ppx) * z_meters / fx
                        y2_cam_white = (cy2_white - ppy) * z_meters / fy
                        z2_cam_white = z_meters
                        
                        point_cam2_white = tf2_geometry_msgs.PointStamped()
                        point_cam2_white.header.frame_id = self.camera_frame
                        point_cam2_white.header.stamp = point_cam.header.stamp
                        point_cam2_white.point.x = x2_cam_white
                        point_cam2_white.point.y = y2_cam_white
                        point_cam2_white.point.z = z2_cam_white

                        try:
                            white_target_point = self.tf_buffer.transform(point_cam, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1))
                            
                            point_base2_white = self.tf_buffer.transform(point_cam2_white, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1))
                            vx_base_white = point_base2_white.point.x - white_target_point.point.x
                            vy_base_white = point_base2_white.point.y - white_target_point.point.y
                            angle_base_white = math.atan2(vy_base_white, vx_base_white)
                            white_target_angle = angle_base_white + math.pi/2 # Align gripper X with short axis

                        except Exception as e:
                            pass

        # --- 2. Detect Green Object (Wood Block) ---
        lower_green = np.array([40, 80, 80]) 
        upper_green = np.array([95, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            if area > 100: 
                # Draw largest contour
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(debug_img, (x + w//2, y + h//2), 5, (255, 0, 0), -1)
                
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # --- Robust Depth Estimation for Green Object ---
                    mask_c = np.zeros_like(cv_depth, dtype=np.uint8)
                    cv2.drawContours(mask_c, [c], -1, 255, -1)
                    valid_depths = cv_depth[mask_c > 0]
                    valid_depths = valid_depths[valid_depths > 0]
                    
                    if len(valid_depths) > 10:
                        depth_val = np.percentile(valid_depths, 5)
                    else:
                        depth_val = cv_depth[cy, cx]
                        if depth_val == 0:
                            for dx in range(-5, 6):
                                for dy in range(-5, 6):
                                    d = cv_depth[min(max(cy+dy, 0), cv_depth.shape[0]-1), min(max(cx+dx, 0), cv_depth.shape[1]-1)]
                                    if d > 0:
                                        depth_val = d
                                        break
                                if depth_val > 0: break
                    
                    if depth_val > 0:
                        z_meters = depth_val * 0.001
                        
                        if z_meters > 1.0:
                             if self.frame_count % 30 == 0:
                                self.get_logger().warn(f'Object too far: {z_meters:.3f}m. Ignoring.')
                             # 发布 debug 图像
                             self._publish_debug_image(debug_img)
                             return
                        
                        # Deproject to 3D (Camera Frame)
                        fx = self.camera_intrinsics.k[0]
                        fy = self.camera_intrinsics.k[4]
                        ppx = self.camera_intrinsics.k[2]
                        ppy = self.camera_intrinsics.k[5]
                        
                        # 标准的相机反投影公式
                        # 注意：OpenCV/ROS光学坐标系中，图像Y向下是正方向
                        x_cam = (cx - ppx) * z_meters / fx
                        y_cam = (cy - ppy) * z_meters / fy  
                        z_cam = z_meters
                        
                        # Log throttling: only log every 10 frames (approx 0.3 sec)
                        should_log = (not self.is_busy) and (self.frame_count % 10 == 0)

                        # Transform to Base Link
                        point_cam = tf2_geometry_msgs.PointStamped()
                        point_cam.header.frame_id = self.camera_frame
                        # 使用Time(0)获取最新可用的TF变换
                        point_cam.header.stamp = rclpy.time.Time().to_msg()
                        point_cam.point.x = x_cam
                        point_cam.point.y = y_cam
                        point_cam.point.z = z_cam
                        
                        try:
                            # 直接查询最新的TF变换
                            point_base = self.tf_buffer.transform(point_cam, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1))
                            
                            # 同时查询当前tool0位置和方向
                            tool0_pos = None
                            tool0_rot = None
                            try:
                                tool0_trans = self.tf_buffer.lookup_transform(self.target_frame, self.tool_link, rclpy.time.Time())
                                tool0_pos = tool0_trans.transform.translation
                                tool0_rot = tool0_trans.transform.rotation
                                tool0_height = tool0_pos.z
                            except Exception as e:
                                self.get_logger().warn(f"无法查询tool0位置: {e}")
                            
                            # 统一打印日志
                            if should_log:
                                self._log_detection_details(cv_depth.shape, cx, cy, ppx, ppy, z_meters, fx, fy, x_cam, y_cam, z_cam, tool0_pos, tool0_rot, point_base)

                            # 记录位置历史
                            self.position_history.append(point_base.point)
                            
                            # 只保留最近10个采样
                            if len(self.position_history) > 10:
                                self.position_history.pop(0)
                            
                            # 检查稳定性
                            max_dev = 0
                            is_stable = False
                            
                            if len(self.position_history) >= 10:
                                xs = [p.x for p in self.position_history]
                                ys = [p.y for p in self.position_history]
                                zs = [p.z for p in self.position_history]
                                max_dev = max(max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs))
                                
                                if max_dev < self.stability_threshold:
                                    is_stable = True

                            if is_stable:
                                # 获取当前 TCP 位姿
                                current_tcp = self.rtde_r.getActualTCPPose()
                                
                                # --- 6D Grasping Orientation Calculation ---
                                try:
                                    # 1. Get Object Angle in Image
                                    rect = cv2.minAreaRect(c)
                                    box = cv2.boxPoints(rect)
                                    box = np.int0(box)
                                    
                                    # Find long edge
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
                                    
                                    # 2. Transform Vector to Base Frame
                                    # Create a second point in image along the angle
                                    cx2 = cx + math.cos(angle_img) * 20 # 20 pixels vector
                                    cy2 = cy + math.sin(angle_img) * 20
                                    
                                    # Deproject P2 (assuming same depth)
                                    x2_cam = (cx2 - ppx) * z_meters / fx
                                    y2_cam = (cy2 - ppy) * z_meters / fy
                                    z2_cam = z_meters
                                    
                                    # Transform P2 to Base
                                    point_cam2 = tf2_geometry_msgs.PointStamped()
                                    point_cam2.header.frame_id = self.camera_frame
                                    point_cam2.header.stamp = point_cam.header.stamp
                                    point_cam2.point.x = x2_cam
                                    point_cam2.point.y = y2_cam
                                    point_cam2.point.z = z2_cam
                                    
                                    point_base2 = self.tf_buffer.transform(point_cam2, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1))
                                    
                                    # Vector in Base Frame
                                    vx_base = point_base2.point.x - point_base.point.x
                                    vy_base = point_base2.point.y - point_base.point.y
                                    
                                    angle_base = math.atan2(vy_base, vx_base)
                                    
                                    # 3. Construct Target Rotation
                                    # Align TCP X-axis with Short Axis of Block (Angle + 90 deg)
                                    # Because gripper fingers are along X, and we want them perpendicular to long axis.
                                    angle_target = angle_base + math.pi/2
                                    
                                    # Rotation Matrix:
                                    # X = [cos(a), sin(a), 0]
                                    # Y = [sin(a), -cos(a), 0]
                                    # Z = [0, 0, -1] (Pointing down)
                                    c_a = math.cos(angle_target)
                                    s_a = math.sin(angle_target)
                                    
                                    R_target = np.array([
                                        [c_a, s_a, 0],
                                        [s_a, -c_a, 0],
                                        [0,   0,  -1]
                                    ])
                                    
                                    rvec, _ = cv2.Rodrigues(R_target)
                                    rx, ry, rz = rvec.flatten()
                                    
                                    # self.get_logger().info(f"Calculated Orientation: Base Angle={math.degrees(angle_base):.1f}, Target Yaw={math.degrees(angle_target):.1f}")
                                    
                                except Exception as e:
                                    self.get_logger().warn(f"Orientation calculation failed: {e}")
                                    rx, ry, rz = current_tcp[3], current_tcp[4], current_tcp[5]
                                    angle_base = 0.0
                                    angle_target = 0.0

                                # 计算目标位置 (Match verify_calibration.py logic)
                                # 在Z轴方向增加15cm保护空间
                                target_pose = [
                                    -point_base.point.x,
                                    -point_base.point.y,
                                    point_base.point.z + 0.15,
                                    rx, ry, rz,
                                ]
                                
                                # 发布 debug 图像 (在等待输入前发布)
                                self._publish_debug_image(debug_img)

                                # Wait for confirmation
                                if not self.is_busy:
                                    # Print detailed info one last time before prompting
                                    self._log_detection_details(cv_depth.shape, cx, cy, ppx, ppy, z_meters, fx, fy, x_cam, y_cam, z_cam, tool0_pos, tool0_rot, point_base)

                                    self.get_logger().info(f"当前 TCP 位姿: {current_tcp}")
                                    self.get_logger().info(f"目标 TCP 位姿 (controller base): {target_pose}")
                                    self.get_logger().info(f"计算抓取角度: Base Angle={math.degrees(angle_base):.1f}, Target Yaw={math.degrees(angle_target):.1f}")
                                    self.get_logger().info("=" * 60)
                                    
                                    if white_target_point:
                                        self.get_logger().info(f"【已检测到蓝色放置目标】: {white_target_point.point}")
                                    else:
                                        self.get_logger().warn("【未检测到蓝色放置目标】")

                                    self.is_busy = True
                                    try:
                                        user_input = input("位置已稳定(10个采样)。是否执行抓取任务? (y/n): ")
                                        if user_input.lower() == 'y':
                                            # Record Start Pose
                                            start_tcp_pose = self.rtde_r.getActualTCPPose()
                                            self.get_logger().info(f"记录起始位姿: {start_tcp_pose}")

                                            # 1. Move to Approach Position (Z+0.15)
                                            self.get_logger().info("1. 移动到预备位置...")
                                            self.rtde_c.moveL(target_pose, 0.05, 0.05)
                                            
                                            # Calculate object width for reference
                                            obj_width_m = w * z_meters / fx
                                            self.get_logger().info(f"检测到物体宽度: {obj_width_m:.4f}m")

                                            # 2. Move Down to Grasp Position (Z)
                                            grasp_pose = list(target_pose)
                                            # Grasp at Centroid: Go 2.5cm below the detected surface
                                            grasp_pose[2] = point_base.point.z - 0.025 
                                            
                                            self.get_logger().info(f"2. 下降到抓取位置 (表面-2.5cm): Z={grasp_pose[2]:.4f}...")
                                            self.rtde_c.moveL(grasp_pose, 0.05, 0.05)

                                            # 3. Close Gripper
                                            self.get_logger().info("3. 闭合夹爪...")
                                            # Command 0.0 (fully closed) to ensure grip with force limit
                                            self.send_gripper_command(0.0)
                                            
                                            # 3.5. Lift to Safe Height (In Place)
                                            self.get_logger().info("3.5. 原地提升至安全高度 (Base Z=0.15m)...")
                                            # Lift to absolute height Z=0.15m in Base Frame
                                            safe_lift_pose = list(grasp_pose)
                                            safe_lift_pose[2] = 0.15
                                            self.rtde_c.moveL(safe_lift_pose, 0.05, 0.05)
                                            
                                            # 4. Place on Blue Object (if detected)
                                            if white_target_point:
                                                self.get_logger().info("4. 移动到蓝色长方体上方放置...")
                                                
                                                place_x = white_target_point.point.x
                                                place_y = white_target_point.point.y
                                                place_z = white_target_point.point.z
                                                
                                                # Calculate Place Orientation (6D)
                                                # Use white_target_angle calculated earlier
                                                c_a = math.cos(white_target_angle)
                                                s_a = math.sin(white_target_angle)
                                                R_place = np.array([
                                                    [c_a, s_a, 0],
                                                    [s_a, -c_a, 0],
                                                    [0,   0,  -1]
                                                ])
                                                rvec_place, _ = cv2.Rodrigues(R_place)
                                                rx_p, ry_p, rz_p = rvec_place.flatten()
                                                
                                                self.get_logger().info(f"放置角度: {math.degrees(white_target_angle):.1f} deg")
                                                self.get_logger().info(f"检测到的表面高度: Z={place_z:.4f}m")

                                                # Approach Pose (Above target)
                                                approach_place_pose = [
                                                    -place_x,
                                                    -place_y,
                                                    place_z + 0.15,
                                                    rx_p, ry_p, rz_p
                                                ]
                                                
                                                # Actual Place Pose (On Surface)
                                                # Place slightly above the surface (2.5cm) to match the grasp offset (assuming centroid grasp)
                                                final_place_pose = [
                                                    -place_x,
                                                    -place_y,
                                                    place_z + 0.025, 
                                                    rx_p, ry_p, rz_p
                                                ]
                                                
                                                self.get_logger().info(f"移动到放置预备位置: {approach_place_pose}")
                                                self.rtde_c.moveL(approach_place_pose, 0.05, 0.05)
                                                
                                                self.get_logger().info(f"移动到放置位置 (表面+2.5cm): {final_place_pose}")
                                                self.rtde_c.moveL(final_place_pose, 0.05, 0.05)
                                                
                                                self.get_logger().info("释放夹爪...")
                                                self.send_gripper_command(0.085)
                                                
                                                self.get_logger().info("抬起...")
                                                self.rtde_c.moveL(approach_place_pose, 0.05, 0.05)
                                                
                                            else:
                                                self.get_logger().info("未检测到蓝色长方体，提升并释放...")
                                                # If no target, lift up and release
                                                lift_pose = list(target_pose)
                                                lift_pose[2] = 0.25 
                                                self.rtde_c.moveL(lift_pose, 0.05, 0.05)
                                                self.send_gripper_command(0.085)
                                            
                                            # 5. Return to Start Pose
                                            self.get_logger().info("5. 返回起始位置...")
                                            self.rtde_c.moveL(start_tcp_pose, 0.05, 0.05)

                                            self.get_logger().info("任务完成。")
                                            self.position_history = []
                                        else:
                                            self.get_logger().info("取消移动，重新开始监测。")
                                            self.position_history = []
                                    finally:
                                        self.is_busy = False
                            
                        except Exception as e:
                            self.get_logger().error(f'TF变换或机械臂控制错误: {e}')
            else:
                self.position_history = [] # 未检测到木块，重置历史
            
            # 发布 debug 图像
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
            self.get_logger().info(f"9. 手眼标定 (tool0 -> camera_color_optical_frame):")
            self.get_logger().info(f"   平移: X=-0.033m, Y=-0.099m, Z=0.008m")
            self.get_logger().info(f"   旋转: qw≈1.0 (几乎无旋转)")
        
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