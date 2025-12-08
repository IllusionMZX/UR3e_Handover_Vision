from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

# 已更新为您的 UR3e IP 地址
ROBOT_IP = "192.168.1.10"

# 目标位姿（base_link 坐标系）
TARGET_POSITION = [0.3926, 0.1871, 0.0290]

# 常见竖直向下姿态：工具 Z 轴指向地面，轴角约 (pi, 0, 0)
VERTICAL_DOWN_ORI = [3.14159, 0.0, 0.0]

LINEAR_SPEED = 0.05  # m/s，低速更安全
LINEAR_ACCEL = 0.05  # m/s^2，进一步放慢加速
STOP_DECEL = 0.2  # m/s^2，用 stopL 平滑减速停止

print(f"正在尝试连接到机械臂 {ROBOT_IP} ...")

# 在运行前，请确认示教器已切换到“远程控制”模式！
try:
    # 建立 RTDE 连接接口
    rtde_c = RTDEControlInterface(ROBOT_IP)
    rtde_r = RTDEReceiveInterface(ROBOT_IP)

    print("连接成功！机械臂已准备好接收指令。")

    # 读取当前 TCP 位姿，仅用于打印参考
    current_tcp = rtde_r.getActualTCPPose()
    print(f"当前 TCP 位姿: {current_tcp}")

    # base 与 base_link 的 X/Y 轴反向：base = -base_link（X、Y）
    target_pose = [
        -TARGET_POSITION[0],
        -TARGET_POSITION[1],
        TARGET_POSITION[2],
        VERTICAL_DOWN_ORI[0],
        VERTICAL_DOWN_ORI[1],
        VERTICAL_DOWN_ORI[2],
    ]
    print(f"目标 TCP 位姿 (base_link): {TARGET_POSITION}")
    print(f"目标 TCP 位姿 (controller base): {target_pose}")

    # moveL 使用笛卡尔空间直线运动
    rtde_c.moveL(target_pose, LINEAR_SPEED, LINEAR_ACCEL)
    print("已到达目标位置。")

    # 平滑停止，避免控制器报 C204A3（路径健全检查）保护停
    rtde_c.stopL(STOP_DECEL)
    print("已平滑减速停止。")

except Exception as e:
    print(f"连接或运动过程中发生错误: {e}")
    # 常见错误：IP错误、防火墙阻止、示教器未启用远程模式。

finally:
    # 确保脚本停止，释放机械臂控制权
    try:
        if 'rtde_c' in locals():
            try:
                rtde_c.stopL(STOP_DECEL)
            except Exception:
                pass
            rtde_c.stopScript()
    except Exception:
        pass
    print("程序已安全退出。")