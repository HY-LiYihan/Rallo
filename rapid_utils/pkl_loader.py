import os
import numpy as np
import pickle

def load_all_pkl_files(frame_dir):
    """
    Load all .pkl files from the given directory and sort them by creation time.
    """
    pkl_files = [
        os.path.join(frame_dir, file)
        for file in os.listdir(frame_dir)
        if file.endswith(".pkl")
    ]

    # Sort files by their creation time
    pkl_files.sort(key=os.path.getctime)

    loaded_data = []
    for pkl_file in pkl_files:
        try:
            with open(pkl_file, "rb") as f:
                data = pickle.load(f)
                loaded_data.append(data)
        except Exception as e:
            print(f"Failed to load file {pkl_file}: {e}")

    return loaded_data

all_datas = load_all_pkl_files("/home/liyh/Rapid_hand/data/test1/")

# 打印第一个数据字典的键
print(all_datas[0].keys())

# 遍历第一个数据字典的项
for key, value in all_datas[0].items():
    print(key, ":", type(value), end="\t")
    if type(value) == np.ndarray:
        print(value.shape)
        # 如果键是 'base_rgb'，则保存数组
        if key == 'base_rgb':
            from PIL import Image

            # 假设 base_rgb 是一个图像数组
            base_rgb_image = Image.fromarray(all_datas[0]['base_rgb'])
            base_rgb_image.save('base_rgb.png')

    elif type(value) == dict:
        print(value)
    print(value)
