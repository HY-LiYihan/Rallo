import time
from rapid_utils.rapidnode import RapidNode
import mujoco
import mujoco.viewer

# 初始化 RapidNode
gello = RapidNode(config_path='args/gello.json', port='/dev/ttyUSB0', init_set=False)

# 加载 MuJoCo 模型
model_path = "third_party/mujoco_menagerie/universal_robots_ur10e/ur10e.xml"  # 替换为你的 MuJoCo XML 文件路径
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

zreopos = [0, -1.57, 1.57, 0, 1.57, 0]
# 主循环：从 RapidNode 读取控制信号并更新 MuJoCo 模型
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 从 RapidNode 读取控制信号
        control_signal = gello.read_pos()
        control_signal+=zreopos
        # print(control_signal)
        if control_signal is not None:
            # 将控制信号应用到 MuJoCo 模型的关节控制值中
            for i, value in enumerate(control_signal):
                if i < model.nu:  # 确保不超出控制维度
                    data.ctrl[i] = value
        # 执行一步模拟
        mujoco.mj_step(model, data)
        viewer.sync()
        # 稍作延迟以避免过快更新
        # time.sleep(0.02)