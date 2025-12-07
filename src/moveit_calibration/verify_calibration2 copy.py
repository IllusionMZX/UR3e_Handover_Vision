#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class CalibrationVerification(Node):
    def __init__(self):
        super().__init__('calibration_verification')
        
        # Parameters
        self.target_frame = 'base_link' 
        self.camera_frame = 'camera_color_optical_frame'
        self.tool_link = 'tool0'
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 1)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 1)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 1)
        
        # Publishers
        self.debug_pub = self.create_publisher(Image, '/calibration_verification/debug_image', 1)
        
        # State
        self.latest_color = None
        self.latest_depth = None
        self.camera_intrinsics = None
        self.frame_count = 0
        
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

        # Detect Green Object
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 80, 80]) 
        upper_green = np.array([95, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Morphological operations
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw contours on image for debugging (for Rviz display)
        debug_img = cv_image.copy()
        
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
                    
                    # Get Depth (Handle 0 depth by neighborhood search)
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
                        
                        self.get_logger().info("=" * 60)
                        self.get_logger().info("【坐标转换详细信息】")
                        self.get_logger().info(f"1. 图像尺寸: {cv_depth.shape[1]}x{cv_depth.shape[0]} 像素")
                        self.get_logger().info(f"2. 检测到的图像坐标: cx={cx}, cy={cy}")
                        self.get_logger().info(f"3. 相机光学中心: ppx={ppx:.1f}, ppy={ppy:.1f}")
                        self.get_logger().info(f"4. 像素偏移量: dx={cx-ppx:.1f}, dy={cy-ppy:.1f}")
                        self.get_logger().info(f"5. 深度值: z={z_meters:.4f}m")
                        self.get_logger().info(f"6. 相机内参: fx={fx:.1f}, fy={fy:.1f}")
                        self.get_logger().info(f"7. 相机坐标系 (camera_color_optical_frame):")
                        self.get_logger().info(f"   X={x_cam:.4f}m, Y={y_cam:.4f}m, Z={z_cam:.4f}m")
                        
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
                            try:
                                tool0_trans = self.tf_buffer.lookup_transform(self.target_frame, self.tool_link, rclpy.time.Time())
                                tool0_pos = tool0_trans.transform.translation
                                tool0_rot = tool0_trans.transform.rotation
                                tool0_height = tool0_pos.z
                                
                                self.get_logger().info("=" * 60)
                                self.get_logger().info("【TF变换链信息】")
                                self.get_logger().info(f"8. 当前 tool0 位置 (相对 base_link):")
                                self.get_logger().info(f"   X={tool0_pos.x:.4f}m, Y={tool0_pos.y:.4f}m, Z={tool0_pos.z:.4f}m")
                                self.get_logger().info(f"   四元数: [{tool0_rot.x:.4f}, {tool0_rot.y:.4f}, {tool0_rot.z:.4f}, {tool0_rot.w:.4f}]")
                                
                                # 手眼标定: tool0 -> camera_color_optical_frame
                                # 平移: [-0.033, -0.099, 0.0076]m
                                # 这意味着相机在tool0的左侧3.3cm，下方9.9cm，前方0.76cm
                                self.get_logger().info(f"9. 手眼标定 (tool0 -> camera_color_optical_frame):")
                                self.get_logger().info(f"   平移: X=-0.033m, Y=-0.099m, Z=0.008m")
                                self.get_logger().info(f"   旋转: qw≈1.0 (几乎无旋转)")
                                
                            except Exception as e:
                                self.get_logger().warn(f"无法查询tool0位置: {e}")
                            
                            self.get_logger().info('='*60)
                            self.get_logger().info(f'🎯 检测到木块!')
                            
                            self.get_logger().info(f"10. base_link 坐标系:")
                            self.get_logger().info(f'    X={point_base.point.x:.4f}m, Y={point_base.point.y:.4f}m, Z={point_base.point.z:.4f}m')
                            self.get_logger().info('='*60)
                            
                        except Exception as e:
                            self.get_logger().error(f'TF变换错误: {e}')
                            
                # 发布 debug 图像
                self._publish_debug_image(debug_img)
    
    def _publish_debug_image(self, cv_image):
        """发布调试图像"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'发布调试图像失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationVerification()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()