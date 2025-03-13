import numpy as np
import xml.etree.ElementTree as ET
import mujoco_py
import transformation_mat
from scipy.spatial.transform import Rotation as R

from LegModel.legs import LegModel

model = mujoco_py.load_model_from_path("../models/dynamic_4l.xml")
sim = mujoco_py.MjSim(model)
length_tail = 0.158

#求身体的质心
def get_CoMofBody(model, sim):
    body_id = model.body_name2id('mouse')
    flag = 0

    #局部坐标
    ##imu在body的坐标, com在body的坐标
    p_imu_body = np.array([0, 0.045, 0.01])
    p_com_body = np.array([0, 0.045, 0])    #y- 0.04
    ##com相对于imu的位置 com - imu
    p_com_imu = p_com_body - p_imu_body
    ##imu的全局坐标 由IMU传感器得到
    p_imu_w = sim.data.sensordata[12:15] - np.array([0,0,0.0157]) #修正高度变化
    ##imu相对于w的旋转
    quat = sim.data.sensordata[15:19] # wxyz顺序
    quat_reordered = np.roll(quat, -1)  # xyzw顺序
    R_imu_w = R.from_quat(quat_reordered).as_matrix() #默认接收的是xyzw
    ## body相对于世界的旋转
    R_body_w = R_imu_w

    #求全局坐标
    ##body全局坐标 也是整个translation
    p_body_w = p_imu_w - (R_imu_w @ p_imu_body)
    translation = p_body_w
    ##com全局坐标
    p_com_w = p_imu_w + (R_imu_w @ p_com_imu)
    mass = model.body_mass[body_id] #0.15

    com = p_com_w
    rotation = R_body_w
    #返回 身体重心，质量，身体框架的旋转矩阵，身体框架的平移
    return com, mass, rotation, translation

#求头部的质心
def get_CoMofHead(model, translation_body_w, rotation_body_w):
    id_bracket = model.body_name2id('mouse_bracket')
    id_head = model.body_name2id('mouse_head')
    mass_bracket = model.body_mass[id_bracket]
    mass_head = model.body_mass[id_head]
    total_mass = mass_bracket + mass_head

    T_body_w = transformation_mat.get_homogeneous_transformation(rotation_body_w, translation_body_w)
    T_bracket_body = transformation_mat.get_transfomation_mat("mouse_bracket")
    T_head_bracket = transformation_mat.get_transfomation_mat("mouse_head")
    T_bracket_body[:3,3] = T_bracket_body[:3,3] + np.array([0,-0.002,0])

    homo_com_bracket = (T_body_w @ T_bracket_body)[:3,3]
    homo_com_head = (T_body_w @ T_bracket_body @ T_head_bracket)[:3,3]

    com_head = (homo_com_bracket[:3] * mass_bracket + homo_com_head[:3] * mass_head)/total_mass
    
    return com_head, total_mass

#求脊柱，需要body在w的translation和rotation，以及spine,常数length和length_yz
def get_CoMofSpine(model, translation_body_w, rotation_body_w, spine, length, length_yz ):
    body_id_t1 = model.body_name2id('t1')
    body_id_t2 = model.body_name2id('t2')
    body_id_t3 = model.body_name2id('t3')
    body_id_t4 = model.body_name2id('t4')
    body_id_hip = model.body_name2id('hip')
    body_id_servo_spine = model.body_name2id('servo_spine')
    mass_t1 = model.body_mass[body_id_t1]
    mass_t2 = model.body_mass[body_id_t2]
    mass_t3 = model.body_mass[body_id_t3]
    mass_t4 = model.body_mass[body_id_t4]
    mass_spine = mass_t1 + mass_t2 + mass_t3 + mass_t4 
    mass_hip = model.body_mass[body_id_hip]
    # mass_spine = 0
    # mass_hip = 0
    mass_servo_spine = model.body_mass[body_id_servo_spine]
    total_mass = mass_hip + mass_spine + mass_servo_spine 

    #身体部分的T
    T_body_w = transformation_mat.get_homogeneous_transformation(rotation_body_w, translation_body_w)
    T_start_body = transformation_mat.get_transfomation_mat("t1") #spine的端点相对于parent
    #Spine起始端点的全局坐标
    pos_start_w = (T_body_w @ T_start_body)[:3,3]  #弧线起始点全局坐标
    #圆心角
    theta = 2 * np.abs(spine)
    #求T_end_start 
    T_end_start = transformation_mat.get_trans_spine_start_to_end(spine, length, length_yz)
    T_start_w = T_body_w @ T_start_body
    T_end_w = T_start_w @ T_end_start
    translation_end_w = T_end_w[:3,3]
    pos_end_w = translation_end_w
    ##按照spine是否为0进行讨论
    if np.abs(theta) < 1e-7:
        com_spine = (pos_start_w + pos_end_w) / 2
    else:
        Radius = length / theta
        x_dash = Radius * np.sin(spine) / spine #圆心到com的距离
        #写出com在起始端点坐标系下的坐标
        if(spine>0):
            translation_com_start = np.array([(Radius - x_dash * np.cos(spine)), np.abs(x_dash * np.sin(spine)), length_yz/2])
        else:
            translation_com_start = np.array([-(Radius - x_dash * np.cos(spine)),  np.abs(x_dash * np.sin(spine)) , length_yz/2])
            
        T_com_start = transformation_mat.get_homo_translation(translation_com_start)
        T_com_w =T_start_w @ T_com_start
        com_spine = T_com_w[:3]

    T_hip_end = transformation_mat.get_transfomation_mat('hip')
    T_hip_w = T_end_w @ T_hip_end
    com_hip = T_hip_w[:3,3]    
        
    #Servo 部分
    T_servo_hip = transformation_mat.get_transfomation_mat('servo_spine')
    T_servo_w = T_hip_w @ T_servo_hip
    com_servo = T_servo_w[:3,3]

    com = (mass_spine * com_spine + mass_hip * com_hip + mass_servo_spine * com_servo) / total_mass

    return com, total_mass, com_hip, T_hip_w

#尾巴部分
def get_CoMofTail(T_hip_w, tail_angle):
    T_tail_main_hip = np.array([0, 0.0217, 0.0028]) # dynamic line 140
    R_tail_main_hip = R.from_euler('x', np.radians(15)).as_matrix()  # 绕 X 轴旋转 15
    trans_tail_main_hip = transformation_mat.get_homogeneous_transformation(R_tail_main_hip, T_tail_main_hip)
    trans_tail_main_world = T_hip_w @ trans_tail_main_hip   
    homo_com_tail_main = trans_tail_main_world @ np.array([0,0.0175,-0.013,1]) #tail line3
    com_tail_main = homo_com_tail_main[:3]

    ##尾巴的计算
    trans_start_main = transformation_mat.get_translation_matrix(np.array([0, 0.03525, -0.0174])) #tail line 10

    translation_mat_com_start = transformation_mat.get_translation_matrix(np.array([0, 0.1493/2, -0.0514/2]))# 长度0.158 -19度偏转
    homo_tail_com = (trans_tail_main_world @ trans_start_main @ translation_mat_com_start)[:3,3] # From tail.xml
    com_tail = homo_tail_com[:3]

    ##电机的计算
    T_servo_main = np.array([0, 0.0152, 0.001]) # tail line 200
    R_servo_main = R.from_euler('x', np.radians(-19)).as_matrix()
    trans_servo_main = transformation_mat.get_homogeneous_transformation(R_servo_main, T_servo_main)
    trans_servo_world = trans_tail_main_world @ trans_servo_main
    homo_com_servo = trans_servo_world[:3,3]
    com_servo = homo_com_servo[:3]

    mass_servo_tail = 0.002
    mass_tail_main = 0.009
    mass_tail = 0.0095
    total_mass = mass_servo_tail + mass_tail + mass_tail_main



    com = (com_tail * mass_tail + com_servo * mass_servo_tail + com_tail_main * mass_tail_main) / total_mass
    return com, total_mass



def get_CoM(model,sim,spine,length,length_yz):

    com_body,m_body,r_b, t_b = get_CoMofBody(model,sim)
    com_head, m_head= get_CoMofHead(model, t_b, r_b)
    com_spine, m_spine, com_hip, T_hip_w = get_CoMofSpine(model, t_b, r_b, spine, length, length_yz)
    com_tail, m_tail = get_CoMofTail(T_hip_w, tail_angle=0)

    total_mass = m_body + m_head + m_spine + m_tail
    
    com_robot = (
        com_body * m_body + com_head * m_head + com_spine * m_spine + com_tail * m_tail
        ) / total_mass
    print("Total Mass: ", total_mass)
    # com_robot = sim.data.sensordata[12:15]

    return com_robot

def get_CoM_Reference(model, sim, flag = 1):
    n_bodies = model.nbody  # 模型中的 body 数量
    weighted_com = np.zeros(3)
    total_mass = 0.0
    for body_id in range(n_bodies):
        mass = sim.model.body_mass[body_id]
        xipos = sim.data.xipos[body_id] 
        # 加权求和
        weighted_com += mass * xipos
        total_mass += mass
    if flag == 0:
        total_com = weighted_com / total_mass
    else:
        total_com = sim.data.subtree_com[model.body_name2id('mouse')]
    
    return total_com

