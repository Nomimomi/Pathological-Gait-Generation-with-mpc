import numpy as np
import xml.etree.ElementTree as ET
import mujoco_py
import transformation_mat
from scipy.spatial.transform import Rotation as R

from LegModel.legs import LegModel
spine_length = 0.034
tail_length = 0.047

model = mujoco_py.load_model_from_path("../models/dynamic_4l.xml")
sim = mujoco_py.MjSim(model)

#求身体的质心
def get_CoMofBody(model, sim):
    body_id = model.body_name2id('mouse')
    flag = 0

    #局部坐标
    ##imu在body的坐标, com在body的坐标
    p_imu_body = np.array([0, 0.045, 0.01])
    p_com_body = np.array([0, 0.04, 0])    #y- 0.04
    ##com相对于imu的位置 com - imu
    p_com_imu = p_com_body - p_imu_body
    ##imu的全局坐标 由IMU传感器得到
    p_imu_w = sim.data.sensordata[12:15]#修正高度变化
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
    correction_com = np.array([0,0.013,0.0023])
    com =  p_com_w + correction_com
    rotation = R_body_w
    #返回 身体重心，质量，身体框架的旋转矩阵，身体框架的平移
    return com, mass, rotation, translation

#求头部的质心
def get_CoMofHead_Body(model):
    id_bracket = model.body_name2id('mouse_bracket')
    id_head = model.body_name2id('mouse_head')
    mass_bracket = model.body_mass[id_bracket]
    mass_head = model.body_mass[id_head]
    total_mass = mass_bracket + mass_head

    p_bracket_body = np.array([0, -0.016, 0.013])
    p_head_body = np.array([0, -0.016, 0.013]) + np.array([0, -0.0106, 0.0144])

    com_head_body = (p_bracket_body * mass_bracket + p_head_body * mass_head)/total_mass

    return com_head_body, total_mass

#求脊柱，需要body在w的translation和rotation，以及spine,常数length和length_yz
def get_CoMofSpine_Body(model, spine):
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
    mass_servo_spine = model.body_mass[body_id_servo_spine]
    total_mass = mass_hip + mass_spine + mass_servo_spine 

    #身体部分的T
    p_start_body = np.array([0, 0.064, 0.026]) #spine的端点相对于parent
    #Spine起始端点的全局坐标
    theta = 2 * np.abs(spine)
    #求T_end_start 
    ##按照spine是否为0进行讨论
    print("Theta:", theta)

    if np.abs(theta) < 1e-9:
        com_spine_body = np.array([0, spine_length/2,-spine_length/2]) + p_start_body
        p_end_body = np.array([0, spine_length, -spine_length]) + p_start_body
        dis_start_end = spine_length
    else:
        Radius = spine_length / theta
        x_dash = Radius * np.sin(spine) / spine #圆心到com的距离
        dis_start_end = np.abs(2 * Radius * np.sin(spine))
        #写出com在起始端点坐标系下的坐标
        if(spine>0):
            translation_com_start = np.array([(Radius - x_dash * np.cos(spine)), np.abs(x_dash * np.sin(spine)), -spine_length/2])
        else:
            translation_com_start = np.array([-(Radius - x_dash * np.cos(spine)),  np.abs(x_dash * np.sin(spine)) , -spine_length/2])

        p_end_body = np.array([dis_start_end * np.sin(spine), dis_start_end*np.cos(spine), -spine_length]) + p_start_body
        com_spine_body = translation_com_start + p_start_body
    print ("distance of spine points: ", dis_start_end)

    p_hip_end = np.array([0, 0.0165, -0.0009])
    com_hip_body = p_end_body + p_hip_end

    p_servo_end = np.array([0, 0.0107, 0])
    com_servo_body = com_hip_body + p_servo_end

    com_spine_total_body = (com_spine_body * mass_spine + com_hip_body * mass_hip + com_servo_body * mass_servo_spine) / total_mass

    return com_spine_total_body, total_mass, com_hip_body

#尾巴部分
def get_CoMofTail_Body(com_hip_body, tail_angle, spine):
    mass_servo_tail = 0.002
    mass_tail_main = 0.009
    mass_tail = 0.0095
    total_mass = mass_servo_tail + mass_tail + mass_tail_main

    com_tail_main_body = com_hip_body + np.array([0, 0.0217, 0.0028])
    com_tail_body = com_hip_body + np.array([(tail_length * np.sin(2*spine))/2, - tail_length * np.cos(2*spine) / 2, tail_length * np.sin(np.radians(-19))/2])
    com_tail_servo_body = com_hip_body + np.array([0, 0.0152, 0.001])

    com_tail_total_body = (com_tail_body * mass_tail + com_tail_main_body * mass_tail_main + com_tail_servo_body * mass_servo_tail) / total_mass

    return com_tail_total_body, total_mass



def get_CoM(model,sim,spine,length,length_yz):
    com_body_w, mass_body, rotation_body_w, translation_body_w = get_CoMofBody(model, sim)
    com_head_body, mass_head = get_CoMofHead_Body(model)
    com_head_w = translation_body_w + (rotation_body_w @ com_head_body)
    com_spine_body, mass_spine, com_hip_body = get_CoMofSpine_Body(model, spine )
    com_spine_w = translation_body_w + (rotation_body_w @ com_spine_body)
    com_tail_body, mass_tail = get_CoMofTail_Body(com_hip_body, 0, spine)
    com_tail_w = translation_body_w + (rotation_body_w @ com_tail_body)

    total_mass = mass_body + mass_head + mass_spine + mass_tail
    total_com = (com_body_w * mass_body + 
                 com_head_w * mass_head + 
                 com_spine_w * mass_spine + 
                 com_tail_w * mass_tail) / total_mass
    # print("Mass:", total_mass)

    return total_com

def get_CoM_Reference(model, sim, flag):
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

