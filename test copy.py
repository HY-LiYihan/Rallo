import numpy as np
import time
from rapid_utils.rapidnode import RalloNode
from ur10_control.UR10_Control import UR10API

# 加载 MuJoCo 模型
ur = UR10API()

zreopos = [0, -1.57, 1.57, 0, 1.57, 3.14]
def calculate_gravity_torque(mass, length, angle_rad):
    """
    计算长方形棍子在给定角度下抵抗重力所需的扭矩。

    Args:
        mass (float): 棍子的质量 (kg).
        length (float): 棍子的长度 (m).
        angle_rad (float): 棍子相对于水平方向的角度 (弧度).

    Returns:
        float: 所需的扭矩 (Nm).
    """
    gravity = 9.81  # 重力加速度 (m/s^2)
    center_of_mass = length / 2.0
    torque = mass * gravity * center_of_mass * np.sin(angle_rad)
    return torque

def torque_to_current(torque, torque_constant):
    """
    将所需的扭矩转换为 Dynamixel 电机所需的电流值。

    Args:
        torque (float): 所需的扭矩 (Nm).
        torque_constant (float): 电机的扭矩常数 (Nm/A).

    Returns:
        float: 电机所需的电流值 (mA)。
    """
    current_amp = torque / torque_constant
    current_ma = current_amp * -100  # 转换为 mA
    return int(current_ma)

if __name__ == '__main__':
    try:
        rallo_node = RalloNode(config_path='args/rallo.json',port="/dev/ttyUSB0")

        # 关节和棍子的物理参数 (你需要根据你的实际情况修改这些值)
        motor_index_2 = 1  # 控制第二个关节 (索引从 0 开始)
        link_mass_2 = 0.10  # 第二个棍子的质量 (kg)
        link_length_2 = 0.5  # 第二个棍子的长度 (m)

        motor_index_3 = 2  # 控制第三个关节 (索引从 0 开始)
        link_mass_3 = 0.1 # 第三个棍子的质量 (kg)
        link_length_3 = 0.1 # 第三个棍子的长度 (m)

        # **重要:** 你需要知道你的 Dynamixel 电机的扭矩常数 (Nm/A)。
        # 这里假设一个示例值，请务必替换为真实值。
        torque_constant = 0.0185  # Nm/A (这是一个示例值，请查阅你的电机规格书)q

        while True:
            # 读取当前关节角度 (单位: rad)
            positions = rallo_node.read_pos()
            control_signal = positions
            control_signal += zreopos
            print(control_signal)
            ur.set_servo_angle_j(angles=control_signal)
            current_angle_z_rad_2 = positions[motor_index_2]
            current_angle_z_rad_3_offset = positions[motor_index_3] - np.pi / 2 - current_angle_z_rad_2

            # **注意角度参考系:**
            # 你之前的公式是基于相对于垂直 z 轴的角度，并使用了 sin 函数。
            # 我这里修改了 `calculate_gravity_torque` 函数，假设 `angle_rad` 是相对于水平方向的角度，并使用了 cos 函数。
            # 你需要根据你的实际机械结构和角度传感器的定义来调整这个函数和角度的计算。

            # 第二个关节的重力补偿
            required_torque_2 = calculate_gravity_torque(link_mass_2, link_length_2, current_angle_z_rad_2+np.pi/8)
            required_current_ma_2 = torque_to_current(required_torque_2, torque_constant)
            print(f"关节 2 - 所需扭矩: {required_torque_2:.4f} Nm, 所需电流: {required_current_ma_2} mA")

            # 第三个关节的重力补偿
            required_torque_3 = calculate_gravity_torque(link_mass_3, link_length_3, current_angle_z_rad_3_offset-np.pi/8)
            required_current_ma_3 = torque_to_current(required_torque_3, torque_constant)
            print(f"关节 3 - 所需扭矩: {required_torque_3:.4f} Nm, 所需电流: {required_current_ma_3} mA")

            # 设置电机的电流 (扭矩控制)
            current_command = [0] * len(rallo_node.motors)  # 创建一个包含所有电机电流的列表
            current_command[motor_index_2] = required_current_ma_2
            current_command[motor_index_3] = required_current_ma_3

            # **重要:** Dynamixel 电机的电流限制需要考虑。
            rallo_node.set_curr(current_command)
            print(f"发送给电机 {motor_index_2} 的电流指令: {required_current_ma_2} mA")
            print(f"发送给电机 {motor_index_3} 的电流指令: {required_current_ma_3} mA")

            # (可选) 读取电机状态进行监控
            velocities = rallo_node.read_vel()
            currents = rallo_node.read_cur()

            print("当前位置 (相对初始):", positions)
            print("当前速度:", velocities)
            print("当前电流:", currents)
            print("-" * 20)

            time.sleep(0.1)

    except Exception as e:
        print("Error:", e)