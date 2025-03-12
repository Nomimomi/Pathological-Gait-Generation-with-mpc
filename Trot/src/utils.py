# utils.py
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def calculate_error(total_steps, ignore_steps, references, evaluations):
    diff_list = []
    for i in range(ignore_steps , total_steps):
        diff = evaluations[i] - references[i]
        diff_list.append(diff)

    squared_errors_x = np.sum(np.abs(diff))    
    return squared_errors_x

def calculate_spine_length_xy():
    bodies = [     
    {"pos": [0, 0.0084, -0.001], "euler": [-11.4, 0, 0]}, # t2
    {"pos": [0, 0.014, -0.00075], "euler": [-6.9, 0, 0]}, # t3
    {"pos": [0, 0.014, -0.00075], "euler": [-5.4, 0, 0]}, # t4
    ]

    total_y_world = 0.0084 + 0.014 * np.cos(np.radians(11.4)) + 0.014 * np.cos(np.radians(18.3))
    print ("length of spine in y: ", total_y_world)
    return total_y_world

def calculate_spine_length_yz():
    bodies = [     
    {"pos": [0, 0.0084, -0.001], "euler": [-11.4, 0, 0]}, # t2
    {"pos": [0, 0.014, -0.00075], "euler": [-6.9, 0, 0]}, # t3
    {"pos": [0, 0.014, -0.00075], "euler": [-5.4, 0, 0]}, # t4
    ]

    total_z_world = 0
    total_z_world = -0.001 - 0.00075 * np.sin(np.radians(11.4)) - 0.00075* np.sin(np.radians(18.3))
    # R_world = np.eye(3)
    # for body in bodies:
    #     pos_local = np.array(body["pos"])  # 读取局部 pos
    #     euler_angles = np.radians(body["euler"])  # 角度转弧度
    #     R_local = R.from_euler('xyz', euler_angles).as_matrix()  # 计算旋转矩阵

    #     # 更新世界坐标系的旋转
    #     R_world = R_world @ R_local  # 累积旋转变换

    #     # 计算当前 body 在世界坐标系中的位移
    #     pos_world = R_world @ pos_local

    #     # 取 y 方向分量
    #     total_z_world += pos_world[2]
    print ("length of spine in z: ", total_z_world)
    return total_z_world