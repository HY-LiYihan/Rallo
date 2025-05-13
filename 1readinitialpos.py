import tkinter as tk
from tkinter import messagebox
from rapid_utils.rapidnode import RapidNode
import time
import json
# import pyperclip  # 用于操作剪贴板

# 初始化 RapidNode
rallo = RapidNode(config_path='args/rallo_init.json', port='/dev/ttyUSB0', init_set=False)

# 创建 Tkinter 窗口
root = tk.Tk()
root.title("关节角显示")

# 创建一个标签列表，用于显示每个关节角
labels = []
for i in range(6):
    label = tk.Label(root, text=f"关节 {i+1}: ", font=("Arial", 12))
    label.pack(pady=5)  # 垂直方向上间隔 5 像素
    labels.append(label)

# 更新关节角的函数
def update_joint_angles():
    try:
        # 读取关节角
        joint_angles = rallo.read_pos()
        if len(joint_angles) == 6:
            for i, angle in enumerate(joint_angles):
                labels[i].config(text=f"关节 {i+1}: {angle:.2f}rad")
        else:
            print("读取到的关节角数量不正确")
    except Exception as e:
        print(f"读取关节角时出错: {e}")

    # 每隔 200 毫秒调用一次自己，更新关节角
    root.after(200, update_joint_angles)

# 保存姿态的函数
def save_pose():
    try:
        # 在接下来的 1 秒内记录 5 次关节角
        joint_angles_list = []
        for _ in range(5):
            joint_angles = rallo.read_pos()
            if len(joint_angles) == 6:
                joint_angles_list.append(joint_angles)
            time.sleep(0.2)  # 每次记录间隔 0.2 秒

        # 计算平均值
        avg_joint_angles = [sum(angles) / len(angles) for angles in zip(*joint_angles_list)]

        # 将平均值保存到剪贴板
        avg_joint_angles_str = str(avg_joint_angles)
        # pyperclip.copy(avg_joint_angles_str)
        print(f"平均关节角已保存到剪贴板: {avg_joint_angles_str}")

        # 弹出对话框询问是否存入 args/rallo.json
        if messagebox.askyesno("保存姿态", "是否将当前姿态存入 args/rallo.json？"):
            # 读取 args/rallo.json 文件
            try:
                with open("args/rallo.json", "r") as f:
                    data = json.load(f)
                    data["init"]["init_pos"] = avg_joint_angles  # 修改 init_pos
                with open("args/rallo.json", "w") as f:
                    json.dump(data, f, indent=4)  # 保存修改后的数据
                messagebox.showinfo("保存成功", "当前姿态已成功存入 args/rallo.json")
            except FileNotFoundError:
                messagebox.showerror("文件未找到", "args/rallo.json 文件不存在")
            except json.JSONDecodeError:
                messagebox.showerror("JSON 解析错误", "args/rallo.json 文件格式错误")
            except Exception as e:
                messagebox.showerror("保存失败", f"保存姿态时出错: {e}")
    except Exception as e:
        messagebox.showerror("保存失败", f"保存姿态时出错: {e}")

# 创建 Save Pose 按钮
save_pose_button = tk.Button(root, text="Save Pose", command=save_pose, font=("Arial", 12))
save_pose_button.pack(pady=10)

# 启动更新关节角的函数
update_joint_angles()

# 运行 Tkinter 主循环
root.mainloop()