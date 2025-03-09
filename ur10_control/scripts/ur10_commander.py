import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
import geometry_msgs.msg

import moveit_py3.srv

# roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch

class UR10Commander(object):
    def __init__(self):
        rospy.wait_for_service("moveit_py3/set_pose")
        self.set_pose_srv = rospy.ServiceProxy("moveit_py3/set_pose", moveit_py3.srv.SetPose)
        rospy.wait_for_service("moveit_py3/get_pose")
        self.get_pose_srv = rospy.ServiceProxy("moveit_py3/get_pose", moveit_py3.srv.GetPose)
        rospy.wait_for_service("moveit_py3/set_positions")
        self.set_positions_srv = rospy.ServiceProxy("moveit_py3/set_positions", moveit_py3.srv.SetPositions)
        rospy.wait_for_service("moveit_py3/get_positions")
        self.get_positions_srv = rospy.ServiceProxy("moveit_py3/get_positions", moveit_py3.srv.GetPositions)
        # joint operation
        rospy.wait_for_service("moveit_py3/set_servo_angle_j")
        self.set_servo_angle_j_srv = rospy.ServiceProxy("moveit_py3/set_servo_angle_j", moveit_py3.srv.SetServoAngleJ)
        rospy.wait_for_service("moveit_py3/get_joint_states")
        self.get_joint_states_srv = rospy.ServiceProxy("moveit_py3/get_joint_states", moveit_py3.srv.GetJointStates)
        rospy.wait_for_service("moveit_py3/get_position")
        self.get_position_srv = rospy.ServiceProxy("moveit_py3/get_position", moveit_py3.srv.GetPosition)                
        


    def set_pose(self, pose, wait=False):
        if wait:
            cur_pose = self.get_pose()
            while np.abs(np.subtract(cur_pose[:3], pose[:3])).max() > 0.005\
                    or np.abs(np.subtract(R.from_quat(cur_pose[3:]).as_matrix(), R.from_quat(pose[3:]).as_matrix())).max() > 0.005:
                # print(np.abs(np.subtract(cur_pose, pose)).max())
                target_pose = geometry_msgs.msg.Pose()
                target_pose.position.x = pose[0]
                target_pose.position.y = pose[1]
                target_pose.position.z = pose[2]
                target_pose.orientation.x = pose[3]
                target_pose.orientation.y = pose[4]
                target_pose.orientation.z = pose[5]
                target_pose.orientation.w = pose[6]
                req = moveit_py3.srv.SetPoseRequest()
                req.target_pose = target_pose
                req.wait = wait
                self.set_pose_srv(req)
                cur_pose = self.get_pose()
        else:
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = pose[0]
            target_pose.position.y = pose[1]
            target_pose.position.z = pose[2]
            target_pose.orientation.x = pose[3]
            target_pose.orientation.y = pose[4]
            target_pose.orientation.z = pose[5]
            target_pose.orientation.w = pose[6]
            req = moveit_py3.srv.SetPoseRequest()
            req.target_pose = target_pose
            req.wait = wait
            self.set_pose_srv(req)


    def get_pose(self):
        res = self.get_pose_srv()
        pose = res.pose
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]

    def set_positions(self, positions, wait=False):
        if wait:
            cur_positions = self.get_positions()
            while np.abs(np.subtract(cur_positions, positions)).max() > 0.1:
                # print(np.abs(np.subtract(cur_positions, positions)).max())
                req = moveit_py3.srv.SetPositionsRequest()
                req.target_positions = positions
                req.wait = wait
                self.set_positions_srv(req)
                cur_positions = self.get_positions()
        else:
            req = moveit_py3.srv.SetPositionsRequest()
            req.target_positions = positions
            req.wait = wait
            self.set_positions_srv(req)


    def get_positions(self):
        res = self.get_positions_srv()
        return res.positions
    
    """--------------------joint operation-------------------------"""
    ## bizt 2024/9/3
    def set_servo_angle_j(self, angles, is_radian=True, speed=None, mvacc=None, wait=False):
        req = moveit_py3.srv.SetServoAngleJRequest()
        req.angles = list(angles)
        req.is_radian = is_radian
        req.speed = 1.0
        req.mvacc = 1.0
        req.wait = wait
        res = self.set_servo_angle_j_srv(req)
        return res.code

    def get_joint_states(self, is_radian=True):
        req = moveit_py3.srv.GetJointStatesRequest()
        req.is_radian = is_radian
        res = self.get_joint_states_srv(req)
        return tuple((res.code, [res.position, res.velocity, res.effort]))

    def get_position(self, is_radian=True):
        req = moveit_py3.srv.GetPositionRequest()
        req.is_radian = is_radian
        res = self.get_position_srv(req)
        return tuple((res.code, res.position))
    

if __name__=="__main__":
    ur10_robot=UR10Commander()
    print(ur10_robot.get_joint_states())