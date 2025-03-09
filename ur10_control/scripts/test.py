from scipy.spatial.transform import Rotation as R
#from ipdb import set_trace
import numpy as np
import os
import json
import time


#from realsense_commander import RealSenseCommander
from ur5_commander import UR5Commander
#from robotiq_gripper import RobotiqGripper
from configs import config

##########################
###### Manipulation ######
##########################

# T_scene_to_world = np.eye(4)
# T_scene_to_world[:3, 3] = [0.76,0.05,0.03]

def tool0_pose_to_cam_transform_matrix(pose, T_cam_to_tool0):
    tool0_pose = pose
    T_tool0_to_world = np.eye(4)
    T_tool0_to_world[:3, :3] = R.from_quat(tool0_pose[3:]).as_dcm()
    T_tool0_to_world[:3, 3] = tool0_pose[:3]
    T_cam_to_world = T_tool0_to_world @ T_cam_to_tool0
    return T_cam_to_world

def cam_transform_matrix_to_tool0_pose(T_cam_to_world, T_cam_to_tool0):
    T_tool0_to_cam = np.linalg.inv(T_cam_to_tool0)
    T_tool0_to_world = T_cam_to_world @ T_tool0_to_cam
    tool0_pose = [] + T_tool0_to_world[:3,3].tolist() + R.from_dcm(T_tool0_to_world[:3,:3]).as_quat().tolist()
    return tool0_pose

def Trans_pose(pose): ### object2cam -> object2world
    T_object_to_cam = np.eye(4)
    T_object_to_cam[:3, 3] = pose
    grasp_matrix = T_cam_to_world @ T_object_to_cam
    grasp_pose = [] + grasp_matrix[:3,3].tolist() + R.from_dcm(grasp_matrix[:3,:3]).as_quat().tolist()
    # grasp_pose[2] += 0.146  # length of gripper
    grasp_pose[2] += 0.151  # length of gripper
    return grasp_pose

### Initialization ###
# cam = RealSenseCommander()
# cam.start()
# print("[ RealSense Initialized ]")

arm = UR5Commander()
init_pose = config.init_position.tolist() + config.init_orientation.tolist()
test_pose = config.test_position.tolist() + config.ori_test.tolist()

# place_pose = config.place_positon.tolist() + config.init_orientation.tolist()
arm.set_pose(init_pose, wait=True)
arm.set_pose(test_pose, wait=True)
# current_pose = arm.get_pose()
# print(current_pose)
# print("Arm Initialized")

# 合理的init pose（仅x，y）
pos = [0.5500550254448288, 1.8328922779303644e-05, 0.8199145991378247, 0.7070319839564942, -0.7071812878557915, 0.00062337711048751, 0.00010568891152561776]
arm.set_pose(pos, wait=True)
# gripper = RobotiqGripper()
# gripper.activation_request()
# gripper.gripper_close()
# print("[ Gripper Initialized ]")
# time.sleep(1)
# set_trace()
# boarder_lt_pose = config.boarder_lt.tolist() + config.init_orientation.tolist()
# boarder_rt_pose = config.boarder_rt.tolist() + config.init_orientation.tolist()
# boarder_lb_pose = config.boarder_lb.tolist() + config.init_orientation.tolist()
# boarder_rb_pose = config.boarder_rb.tolist() + config.init_orientation.tolist()
# arm.set_pose(boarder_lt_pose, wait=True)
# time.sleep(3)
# arm.set_pose(boarder_rt_pose, wait=True)
# time.sleep(3)
# arm.set_pose(boarder_lb_pose, wait=True)
# time.sleep(3)
# arm.set_pose(boarder_rb_pose, wait=True)
# time.sleep(3)




#set_trace()

with open(os.path.join(os.path.dirname(__file__), "../config/cam_info_realsense.json"), 'r') as f:
    cam_info = json.load(f)

intrinsics = np.array(cam_info["K"]).reshape(3, 3)
T_cam_to_tool0 = np.array(cam_info["cam_to_tool0"]).reshape(4, 4)

init_matrix = np.eye(4)
init_matrix[:3, :3] = R.from_quat(init_pose[3:]).as_matrix()
init_matrix[:3, 3] = init_pose[:3]

T_cam_to_world = init_matrix @ T_cam_to_tool0

print(T_cam_to_world)