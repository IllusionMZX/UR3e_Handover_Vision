#!/usr/bin/env python3
"""
简单的位置移动脚本
使用MoveIt Action接口移动到base_link坐标系下的目标位置
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
import sys

class SimplePositionMover(Node):
    def __init__(self):
        super().__init__('simple_position_mover')
        
        # Action Client
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # 参数
        self.target_frame = 'base_link'
        self.planning_group = 'ur_manipulator'
        self.tool_link = 'tool0'
        
        self.get_logger().info('✓ 等待MoveGroup action服务器...')
        self._action_client.wait_for_server()
        self.get_logger().info('✓ MoveGroup已连接')
        
    def move_to_position(self, x, y, z):
        """
        移动到base_link坐标系下的XYZ位置（只约束位置）
        
        参数:
            x, y, z: 目标位置坐标（米）
        """
        self.get_logger().info(f'目标位置: X={x:.4f}m, Y={y:.4f}m, Z={z:.4f}m')
        
        # 创建MoveGroup goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1  # 10% 速度
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        goal_msg.request.planner_id = "RRTConnect"
        
        # 工作空间
        goal_msg.request.workspace_parameters.header.frame_id = self.target_frame
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -0.5
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0
        
        # 创建位置约束（球形区域，半径3cm）
        goal_constraints = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.target_frame
        pos_constraint.header.stamp = self.get_clock().now().to_msg()
        pos_constraint.link_name = self.tool_link
        pos_constraint.weight = 1.0
        
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.03]  # 3cm半径
        
        sphere_pose = Pose()
        sphere_pose.position.x = x
        sphere_pose.position.y = y
        sphere_pose.position.z = z
        sphere_pose.orientation.w = 1.0
        
        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(sphere_pose)
        
        goal_constraints.position_constraints.append(pos_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        # 发送goal
        self.get_logger().info('开始规划...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('✗ 目标被拒绝')
            return
        
        self.get_logger().info('✓ 目标已接受，规划中...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val
        
        if error_code == 1:
            self.get_logger().info('✓ 移动完成!')
        else:
            error_msgs = {
                -1: '规划失败',
                -2: '无效的目标状态',
                -3: '超时',
                -10: '没有找到IK解',
                -21: '碰撞检测失败'
            }
            error_msg = error_msgs.get(error_code, f'未知错误: {error_code}')
            self.get_logger().error(f'✗ 移动失败: {error_msg}')

def main(args=None):
    rclpy.init(args=args)
    
    # 从命令行参数获取目标位置
    if len(sys.argv) != 4:
        print("用法: python3 move_to_position.py <x> <y> <z>")
        print("示例: python3 move_to_position.py 0.3926 0.1871 0.2290")
        return
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    
    node = SimplePositionMover()
    
    # 移动到目标位置
    node.move_to_position(x, y, z)
    
    # 保持节点运行以接收回调
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
