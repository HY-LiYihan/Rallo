# from .scripts.ur10_commander import UR10Commander
import numpy as np
import ctypes
import cv2
import sys
import os
import time


class UR10API(object):

    def __init__(self):
        from .scripts.ur10_commander import UR10Commander

        self.arm = UR10Commander()
        print("hello")

    # 描述：设置关节角度，入参：angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs，出参：code
    def set_servo_angle_j(
        self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs
    ):
        code = self.arm.set_servo_angle_j(angles)
        return code

    # 描述：得到当前关节角度，入参：is_radian=None, num=3，出参：tuple((code, [position, velocity, effort]))
    def get_joint_states(self, is_radian=None, num=3):
        return self.arm.get_joint_states()

    # 描述：得到当前位置，入参：is_radian=None，出参：tuple((code, [x, y, z, roll, pitch, yaw]))
    def get_position(self, is_radian=None):
        return self.arm.get_position()

