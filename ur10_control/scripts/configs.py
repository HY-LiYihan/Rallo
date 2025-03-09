import os
import numpy as np
from scipy.spatial.transform import Rotation as R

DEVICE = "cuda:0"
EPSILON = 1e-6

class Config:
    def __init__(self):
        self.package_name = "psdf_suction"
        self.path = os.path.dirname(os.path.dirname(__file__))

        # vaccum cup
        self.gripper_radius = 0.01
        self.gripper_height = 0.02
        self.gripper_vertices = 8
        self.gripper_angle_threshold = 45
        self.vacuum_length = 0.125

        # PSDF
        self.volume_origin = np.array([0.0, -0.25, 0.02])
        self.volume_range = np.array([0.5, 0.5, 0.5])
        self.volume_resolution = 0.002
        self.volume_shape = np.ceil(self.volume_range / self.volume_resolution).astype(np.int32).tolist()
        self.T_volume_to_world = np.eye(4).astype(np.float32)
        self.T_volume_to_world[:3, 3] = self.volume_origin
        self.T_world_to_volume = np.eye(4).astype(np.float32)
        self.T_world_to_volume[:3, 3] = -self.T_volume_to_world[:3, 3]

        # setting init pose
        self.init_position = (2 * self.volume_origin + self.volume_range) / 2
        self.init_position[2] = self.volume_origin[2] + self.volume_range[2]
        self.init_orientation = R.from_euler("xyz", [180, 0., -90], degrees=True).as_quat()  ### tool02world
        # which camera is on top of the middle of workspace
        # self.init_cam_height = 0.35
        # mid_point = (self.upper_bound + self.lower_bound) / 2
        # T_cam2world = np.eye(4)
        # T_cam2world[:3, :3] = R.from_euler("xyz", [180, 0.1, -90], degrees=True).as_matrix()# + np.eye(3) * 1e-3
        # # print(np.linalg.det(T_cam2world[:3, :3]))
        # T_cam2world[:3, 3] = mid_point
        # T_cam2world[2, 3] = self.init_cam_height
        # T_cam2tool0 = np.loadtxt(
        #     os.path.join(os.path.dirname(__file__), "../config/camera_pose_base_tip.txt"))
        # T_tool02world = T_cam2world @ np.linalg.inv(T_cam2tool0)
        # self.init_position = T_tool02world[:3, 3]
        # self.init_quaternion = R.from_matrix(T_tool02world[:3, :3]).as_quat()
        # self.init_pose = self.init_position.tolist() + self.init_quaternion.tolist()
        self.init_position = self.init_position + np.array([0.3, 0, 0.15])
        # self.init_position = self.init_position + np.array([0.3, 0, 0.3]) # 原始
        self.place_position = self.init_position.copy() + np.array([0, 0.4, -0.3])
        # self.place_positon = self.init_position + np.array([-0.5, 0.4, -0.5])
        # self.place_positon = self.init_position + np.array([-0.5, 0.5, -0.5])
        ### test
        self.rotation = R.from_euler("xyz", [0, 0., -45], degrees=True).as_matrix() @ R.from_quat(self.init_orientation.copy()).as_matrix()
        self.ori_test = R.from_matrix(self.rotation).as_quat()
        self.test_position = self.init_position.copy() + np.array([0.3, 0, 0])
        # boarder test
        self.grasp_position = self.init_position.copy() + np.array([0.1, 0, 0])
        self.boarder_lt = self.grasp_position.copy() + np.array([0.224, 0.224, -0.54])
        self.boarder_rt = self.grasp_position.copy() + np.array([0.224, -0.224, -0.54])
        self.boarder_lb = self.grasp_position.copy() + np.array([-0.224, 0.224, -0.54])
        self.boarder_rb = self.grasp_position.copy() + np.array([-0.224, -0.224, -0.54])
        

config = Config()