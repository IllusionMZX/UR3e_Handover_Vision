#!/usr/bin/env python3
"""
使用URScript直接控制机械臂移动到目标位置
通过ur_dashboard发送URScript命令实现直线运动
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ur_dashboard_msgs.srv import GetRobotMode
from ur_dashboard_msgs.msg import RobotMode
import sys
import time

class URScriptMover(Node):
    def __init__(self):
        super().__init__('urscript_mover')
        
        # 创建发布者，发送URScript到机器人
        self.script_publisher = self.create_publisher(
            String, 
            '/urscript_interface/script_command',  # UR driver的URScript接口
            10
        )
        
        # 等待连接
        time.sleep(0.5)
        self.get_logger().info('✓ URScript发布器已准备')
        
    def move_to_position(self, x, y, z, velocity=0.1, acceleration=0.3):
        """
        使用movel移动到base_link坐标系下的XYZ位置（直线运动）
        保持当前姿态不变
        
        参数:
            x, y, z: 目标位置坐标（米）
            velocity: 速度 (m/s)，默认0.1m/s = 10cm/s
            acceleration: 加速度 (m/s²)，默认0.3m/s²
        """
        self.get_logger().info('='*60)
        self.get_logger().info(f'目标位置: X={x:.4f}m, Y={y:.4f}m, Z={z:.4f}m')
        self.get_logger().info(f'运动速度: {velocity}m/s, 加速度: {acceleration}m/s²')
        
        # 构建URScript命令
        # movel(pose, a=1.2, v=0.25, t=0, r=0)
        # pose格式: [x, y, z, rx, ry, rz]
        # 这里我们只改变xyz，保持当前的rx,ry,rz（通过get_actual_tcp_pose()获取）
        
        urscript = f"""# 获取当前位姿
current_pose = get_actual_tcp_pose()

# 构造目标位姿（保持当前姿态，只改变位置）
target_pose = p[{x}, {y}, {z}, current_pose[3], current_pose[4], current_pose[5]]

# 使用movel进行直线运动
movel(target_pose, a={acceleration}, v={velocity})

textmsg("到达目标位置")
"""
        
        self.get_logger().info('发送URScript命令...')
        self.get_logger().info(f'URScript:\n{urscript}')
        
        # 发布URScript命令
        msg = String()
        msg.data = urscript
        self.script_publisher.publish(msg)
        
        self.get_logger().info('✓ 命令已发送，机械臂正在移动...')
        self.get_logger().info('='*60)

def main(args=None):
    rclpy.init(args=args)
    
    # 从命令行参数获取目标位置
    if len(sys.argv) < 4:
        print("用法: python3 move_with_urscript.py <x> <y> <z> [velocity] [acceleration]")
        print("示例: python3 move_with_urscript.py 0.3926 0.1871 0.2290 0.05 0.2")
        print("      (移动到目标位置，速度5cm/s，加速度0.2m/s²)")
        print("\n默认值: velocity=0.1m/s (10cm/s), acceleration=0.3m/s²")
        return
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    
    # 可选参数
    velocity = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1
    acceleration = float(sys.argv[5]) if len(sys.argv) > 5 else 0.3
    
    node = URScriptMover()
    
    # 移动到目标位置
    node.move_to_position(x, y, z, velocity, acceleration)
    
    # 保持节点运行一段时间让命令发送出去
    time.sleep(2.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
