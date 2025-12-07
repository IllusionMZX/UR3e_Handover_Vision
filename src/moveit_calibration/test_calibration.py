#!/usr/bin/env python3
"""
测试手眼标定是否正确
"""
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np

class CalibrationTest(Node):
    def __init__(self):
        super().__init__('calibration_test')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 等待TF
        import time
        time.sleep(2.0)
        
        self.test_calibration()
        
    def test_calibration(self):
        print("\n" + "="*70)
        print("手眼标定验证测试")
        print("="*70)
        
        try:
            # 查询手眼标定TF
            trans = self.tf_buffer.lookup_transform('wrist_3_link', 'camera_color_optical_frame', rclpy.time.Time())
            
            t = trans.transform.translation
            r = trans.transform.rotation
            
            print("\n手眼标定数据 (wrist_3_link -> camera_color_optical_frame):")
            print(f"  平移: X={t.x:.4f}, Y={t.y:.4f}, Z={t.z:.4f} 米")
            print(f"  旋转: X={r.x:.4f}, Y={r.y:.4f}, Z={r.z:.4f}, W={r.w:.4f}")
            
            # 计算欧拉角
            import math
            # 简化的欧拉角计算 (roll, pitch, yaw)
            roll = math.atan2(2*(r.w*r.x + r.y*r.z), 1 - 2*(r.x**2 + r.y**2))
            pitch = math.asin(2*(r.w*r.y - r.z*r.x))
            yaw = math.atan2(2*(r.w*r.z + r.x*r.y), 1 - 2*(r.y**2 + r.z**2))
            
            print(f"  欧拉角: Roll={math.degrees(roll):.1f}°, Pitch={math.degrees(pitch):.1f}°, Yaw={math.degrees(yaw):.1f}°")
            
            print("\n分析:")
            # 对于手在眼上配置
            print("  配置类型: 手在眼上 (Eye-in-Hand)")
            
            # 检查Z坐标
            if abs(t.z) < 0.05:
                print(f"  ⚠️  Z偏移很小 ({t.z:.3f}m)，相机可能与手腕在同一平面")
            elif t.z > 0.05:
                print(f"  ✓  相机在手腕上方 {t.z*100:.1f}cm")
            else:
                print(f"  ✓  相机在手腕下方 {abs(t.z)*100:.1f}cm")
            
            # 检查旋转
            if abs(roll) < 10 and abs(pitch) < 10 and abs(yaw) < 10:
                print(f"  ⚠️  旋转很小，相机可能与手腕方向几乎一致")
                print(f"     这对于朝下看物体可能不合适")
            
            # 模拟测试：假设物体在相机前方30cm
            print("\n" + "-"*70)
            print("模拟测试: 假设木块在相机前方30cm (相机坐标系: Z=0.3m)")
            print("-"*70)
            
            # 相机坐标系中的点
            cam_point = np.array([0.0, 0.0, 0.3, 1.0])  # [x, y, z, 1]
            
            # 构造变换矩阵
            # 从四元数转旋转矩阵
            x, y, z, w = r.x, r.y, r.z, r.w
            R = np.array([
                [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
                [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
                [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
            ])
            
            T_wrist_cam = np.eye(4)
            T_wrist_cam[:3, :3] = R
            T_wrist_cam[:3, 3] = [t.x, t.y, t.z]
            
            # 变换到手腕坐标系
            wrist_point = T_wrist_cam @ cam_point
            
            print(f"  相机坐标系: X=0.000, Y=0.000, Z=0.300 米")
            print(f"  手腕坐标系: X={wrist_point[0]:.3f}, Y={wrist_point[1]:.3f}, Z={wrist_point[2]:.3f} 米")
            
            # 假设手腕在base_link上方50cm
            print("\n假设手腕当前位置 (base_link): X=0.4, Y=0.2, Z=0.5 米")
            base_wrist = np.array([0.4, 0.2, 0.5])
            base_object = base_wrist + wrist_point[:3]
            
            print(f"  → 木块在base_link坐标系: X={base_object[0]:.3f}, Y={base_object[1]:.3f}, Z={base_object[2]:.3f} 米")
            
            if base_object[2] < 0:
                print("\n❌ 严重问题: 木块Z坐标为负，在地面以下！")
                print("   这说明手眼标定有错误，可能：")
                print("   1. 标定时相机光轴方向配置错误")
                print("   2. 标定数据采集有问题")
                print("   3. 需要重新进行手眼标定")
            elif base_object[2] > 0.6:
                print("\n⚠️  木块Z坐标很高，可能超出工作范围")
            else:
                print("\n✓ 木块Z坐标合理 (在桌面上)")
                
        except Exception as e:
            print(f"\n❌ 错误: {e}")
            print("   请确认:")
            print("   1. UR机器人驱动已启动")
            print("   2. camerapose2.launch.py 已启动")
            print("   3. RealSense相机驱动已启动")
        
        print("\n" + "="*70 + "\n")

def main():
    rclpy.init()
    node = CalibrationTest()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
