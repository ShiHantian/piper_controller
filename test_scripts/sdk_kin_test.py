#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
from piper_sdk import *
import numpy as np

def rxryrz_to_rotation_matrix(rx, ry, rz):
    """Fixed angles to rotation matrix
    (refer to a fixed fram, rotate around its x, y, z axis in sequence)
    Input: rx, ry, rz in radians
    Output: 3x3 rotation matrix"""

    cos_rz = np.cos(rz)
    sin_rz = np.sin(rz)
    cos_ry = np.cos(ry)
    sin_ry = np.sin(ry)
    cos_rx = np.cos(rx)
    sin_rx = np.sin(rx)

    matrix = np.array([
        [cos_rz*cos_ry, cos_rz*sin_ry*sin_rx-sin_rz*cos_rx, cos_rz*sin_ry*cos_rx+sin_rz*sin_rx],
        [sin_rz*cos_ry, sin_rz*sin_ry*sin_rx+cos_rz*cos_rx, sin_rz*sin_ry*cos_rx-cos_rz*sin_rx],
        [-sin_ry, cos_ry*sin_rx, cos_ry*cos_rx]
    ])

    return matrix

def xyzrxryrz_to_transform_matrix(x, y, z, rx, ry, rz):
    """Convert translation and rotation angles to 4x4 transformation matrix
    Input: x, y, z position and rx, ry, rz rotation angles in radians
    Output: 4x4 homogeneous transformation matrix"""
    
    # Get the rotation matrix
    rotation_matrix = rxryrz_to_rotation_matrix(rx, ry, rz)
    
    # Create the 4x4 transformation matrix
    transform_matrix = np.eye(4)
    transform_matrix[0:3, 0:3] = rotation_matrix
    transform_matrix[0:3, 3] = [x, y, z]
    
    return transform_matrix

# 测试代码
if __name__ == "__main__":
    piper = C_PiperInterface_V2(dh_is_offset=1)
    piper.ConnectPort()
    # 使用前需要使能
    piper.EnableFkCal()
    # 设置numpy打印选项，禁止科学计数法，并保留两位小数
    np.set_printoptions(suppress=True, precision=2)
    # 注意，由于计算在单一线程中十分耗费资源，打开后会导致cpu占用率上升接近一倍
    while True:
        # 反馈6个浮点数的列表，表示 1-6 号关节的位姿
        fk_feedback = piper.GetFK('feedback')
        if fk_feedback:
            print("--- Feedback FK Transform Matrices ---")
            for i, pose in enumerate(fk_feedback):
                if len(pose) == 6:
                    # 单位转换
                    # XYZ in mm
                    x, y, z = pose[0], pose[1], pose[2]
                    # RX, RY, RZ to radians
                    rx = np.deg2rad(pose[3])
                    ry = np.deg2rad(pose[4])
                    rz = np.deg2rad(pose[5])

                    # 获取变换矩阵
                    transform_matrix = xyzrxryrz_to_transform_matrix(x, y, z, rx, ry, rz)

                    # 格式化打印
                    print(f"--- Joint {i+1} Transform Matrix ---")
                    print(transform_matrix)
                else:
                    print(f"--- Joint {i+1} has malformed pose data: {pose} ---")
            print("-" * 40)
        else:
            print("Could not get FK feedback data.")
        
        time.sleep(2)