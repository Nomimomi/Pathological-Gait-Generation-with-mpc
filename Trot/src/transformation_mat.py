import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R

tree = ET.parse('../models/dynamic_4l.xml')
root = tree.getroot()
factor = 0.28
# 通过xml文件获得transformation matrix
def get_transfomation_mat(bodyname):
    for body in root.iter('body'):
        if body.get('name') == bodyname:
            pos_str = body.get('pos')
            euler_str = body.get('euler')
            pos = np.array([float(i) for i in pos_str.split()])
            if euler_str:
                euler = np.array([float(i) for i in euler_str.split()])
            else:
                euler = np.array([0.0, 0.0, 0.0])

    rotation_matrix = R.from_euler('xyz', euler).as_matrix()

    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix  # 前 3x3 部分是旋转矩阵
    transform_matrix[:3, 3] = pos  # 最后一列是平移向量

    # print("Transformation Matrix:")
    # print(transform_matrix)

    return transform_matrix
    

def get_trans_spine_start_to_end(spine, length, length_yz):
    transform_matrix = np.eye(4)
    theta_x = np.radians(-23.7)
    theta = 2 * spine
    rotation = np.eye(3)
    length = factor * length
    length_yz = factor * length_yz
    print ("theta: " ,theta)
    if np.abs(spine) > 1e-6:
        Radius = np.abs(length / theta) 
        dis_start_to_end = np.abs(2 * Radius * np.sin(spine)) #两个端点的直线距离
        T_x = dis_start_to_end * np.sin(spine)
        T_y = dis_start_to_end * np.cos(spine)
        rotation = R.from_euler('zx',[-theta, theta_x]).as_matrix()
        # rotation = R.from_euler('xz', [theta_x, -theta]).as_matrix()
        translation = np.array([T_x, T_y, length_yz])
    else:
        rotation = R.from_euler('x',theta_x).as_matrix()
        translation = np.array([0, length, length_yz])

    transform_matrix[:3, :3] = rotation
    transform_matrix[:3, 3] = translation

    return transform_matrix

# def get_trans_spine_start_to_end(spine, length, length_yz):
#     transform_matrix = np.eye(4)
#     theta_x = np.radians(-23.7)

#     if np.abs(spine) > 1e-6:
#         Radius = np.abs(length / (2 * spine)) 
#         dis_start_to_end = np.abs(2 * Radius * np.sin(spine)) #两个端点的直线距离
#         T_x = dis_start_to_end * np.sin(spine)
#         T_y = dis_start_to_end * np.cos(spine)
#         rotation = R.from_euler('xz',[ theta_x, -2 * spine]).as_matrix()
#         # rotation = R.from_euler('xz',[theta_x, 2 * spine]).as_matrix()
#         translation = np.array([T_x, T_y, length_yz/2])
#     else:
#         rotation = np.eye(3)
#         rotation = R.from_euler('x',theta_x).as_matrix()
#         translation = np.array([0, length, length_yz/2])

#     transform_matrix[:3, :3] = rotation
#     transform_matrix[:3, 3] = translation

#     return transform_matrix

# 将R + T合并为4x4的transformation matrix
def get_homogeneous_transformation(rotation, translation):
    # 创建 4x4 的单位矩阵
    transform_matrix = np.eye(4)

    # 将旋转矩阵放入 transform_matrix 的左上角
    transform_matrix[:3, :3] = rotation

    # 将平移向量放入 transform_matrix 的最后一列
    transform_matrix[:3, 3] = translation

    return transform_matrix

# 3x1 -> 4x1
def get_homo_translation(translation):
    return np.append(translation,1)

# 3x1 -> 4x4 （旋转矩阵为eye(3)）  #只有平移
def get_translation_matrix(translation):
    transformation = np.eye(4)
    transformation[:3, 3] = translation
    return transformation

def get_rotation_matrix(euler):
    if isinstance(euler, str):
        euler = [float(x) for x in euler.split()]
    else:
        euler = list(euler)
    
    if len(euler) != 3:
        raise ValueError("Euler angles must have 3 components")
    angles = np.radians(euler)
    rotation = R.from_euler('xyz', angles)
    rotation = rotation.as_matrix()
    return rotation