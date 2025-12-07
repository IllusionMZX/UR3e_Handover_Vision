#!/usr/bin/env python3
"""检查相机坐标系方向"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import tf2_ros

class CameraFrameChecker(Node):
    def __init__(self):
        super().__init__('camera_frame_checker')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 订阅相机信息
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 1)
        
        import time
        time.sleep(2)
        
        self.check_frames()
        
    def info_callback(self, msg):
        print(f"\n相机内参信息:")
        print(f"  分辨率: {msg.width} x {msg.height}")
        print(f"  Frame ID: {msg.header.frame_id}")
        
    def check_frames(self):
        print("\n" + "="*70)
        print("检查RealSense相机坐标系")
        print("="*70)
        
        # 检查所有可能的相机frame
        possible_frames = [
            'camera_link',
            'camera_color_frame', 
            'camera_color_optical_frame',
            'camera_depth_frame',
            'camera_depth_optical_frame'
        ]
        
        print("\n可用的相机Frame:")
        for frame in possible_frames:
            try:
                if self.tf_buffer.can_transform('tool0', frame, rclpy.time.Time()):
                    trans = self.tf_buffer.lookup_transform('tool0', frame, rclpy.time.Time())
                    t = trans.transform.translation
                    print(f"  ✓ {frame}")
                    print(f"    相对tool0: X={t.x:.3f}, Y={t.y:.3f}, Z={t.z:.3f} 米")
            except:
                pass
        
        print("\n" + "="*70)
        print("RealSense坐标系说明:")
        print("="*70)
        print("camera_color_frame (非光学坐标系):")
        print("  X: 右, Y: 下, Z: 前(相机朝向)")
        print("\ncamera_color_optical_frame (光学坐标系 - OpenCV/ROS标准):")
        print("  X: 右, Y: 下, Z: 前(深度方向)")
        print("\n你当前使用: camera_color_optical_frame")
        print("深度数据应该在Z轴上")
        print("="*70 + "\n")

def main():
    rclpy.init()
    node = CameraFrameChecker()
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
