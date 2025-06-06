import time
from rapid_utils.rapidnode import RapidNode

# from ur10_control.UR10_Control import UR10API

# 初始化 RapidNode
rallo = RapidNode(config_path="args/rallo.json", port="/dev/ttyUSB0", init_set=False)

# 加载 MuJoCo 模型
# ur = UR10API

zreopos = [0, -1.57, 1.57, 0, 1.57, 0]
# 主循环：从 RapidNode 读取控制信号并更新 MuJoCo 模型
while True:
    # 从 RapidNode 读取控制信号
    control_signal = rallo.read_pos()
    print("cmp_pos2zeropos",control_signal)
    control_signal += zreopos
    print("real_send_pos",control_signal)
    # ur.set_servo_angle_j(control_signal)
    # 稍作延迟以避免过快更新
    time.sleep(0.1)
