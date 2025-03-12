import numpy as np
import xml.etree.ElementTree as ET
import mujoco_py
import transformation_mat
from scipy.spatial.transform import Rotation as R

from LegModel.legs import LegModel

##TODO
model = mujoco_py.load_model_from_path("../models/dynamic_4l.xml")
sim = mujoco_py.MjSim(model)
# 获得body的全局坐标
def get_CoMofBody(model,sim):
    body_id = model.body_name2id('mouse')
    flag = 0

    #局部坐标
    p_imu_mouse = np.array([0, 0.045, 0.01])  # IMU在 "mouse"坐标系中的局部位置     
    p_com_mouse = np.array([0, 0.04, 0])      # COM在 "mouse" 坐标系中的局部位置    p_imu_body = np.array([0, 0.045, 0.01]) - np.array([0, 0.04, 0])
    p_com_imu = p_com_mouse - p_imu_mouse     # COM 相对imu的偏移
    p_imu_w = sim.data.sensordata[12:15]   #IMU 的世界坐标
    
    #quat 转换为 rotation imu在世界坐标下的旋转
    quat = sim.data.sensordata[15:19] # [w,x,y,z]
    quat_reordered = np.roll(quat, -1)  # [x, y, z, w]
    R_imu_w = R.from_quat(quat_reordered).as_matrix() #from_quat 默认接收的是[x,y,z,w]

    #直接获得"mouse"的rotation和Pos
    rotation_body_world = sim.data.body_xmat[body_id].reshape(3,3)
    
    #mouse原点的世界坐标 用作后面转移矩阵的tranlation
    # bodyframe in world
    p_mouse_w = p_imu_w - R_imu_w @ p_imu_mouse # body在世界坐标系的坐标
    # 质心在世界坐标系中的位置
    p_com_w = p_imu_w + R_imu_w @ p_com_imu #+ np.array([0.9e-6, 0.0454,0])
    com_sim = sim.data.xipos[body_id] 
    mass = model.body_mass[body_id]
    # p_com_w = p_com_w + np.array([-0.001, 0.0250,0])
    # subtree_com_mouse = sim.data.subtree_com[body_id]
    # 打印全局坐标
    # print ("身体坐标系转换：", R_imu_w)
    # print ("计算出的质心: ", p_com_w)
    # print ("xipos ", com_sim)
    # print ("subtree com of body: ",subtree_com_mouse)
    # print ("transformation matrix BODY IN WORLD imu: ", R_imu_w) 
    # print ("transformation matrix BODY IN WORLD: ", rotation_body_world) 
    if flag == 0:
        com = p_com_w
        rotation = R_imu_w
    else: 
        com = com_sim  #body的com的全局坐标
        rotation = rotation_body_world
    
    return com, mass, rotation, p_mouse_w
    # print(rotation_matrix.T @ rotation_body_world)  # 如果接近单位矩阵，则只是参考系不同


#head全局
def get_CoMofHead(model, translation, rotation_parent_world):
    # include_bodies = ["mouse_bracket","mouse_head"]
    id1 = model.body_name2id('mouse_bracket')
    id2 = model.body_name2id('mouse_head')
    mass1 = model.body_mass[id1]
    mass2 = model.body_mass[id2]
    total_mass = mass1 + mass2 #
    # translation = translation #+ np.array([0, -0.04, 0])
    # homo_com_body = np.append(com_parent, 1)
    trans_body_world = transformation_mat.get_homogeneous_transformation(rotation_parent_world, translation)
    trans_bracket_body = transformation_mat.get_transfomation_mat("mouse_bracket")
    trans_head_bracket = transformation_mat.get_transfomation_mat("mouse_head")
    homo_com_bracket = (trans_body_world @ trans_bracket_body)[:3, 3] #mesh的com如果geom的pos为0，那么就是body的局部坐标系的原点
    homo_com_head =(trans_body_world @ trans_bracket_body@ trans_head_bracket)[:3, 3]
    
    print ("transformation matrix BODY IN WORLD: ", trans_body_world) 
    # print ("subtree com of head: ",com_subtree_head )
    # bracket和head与上一级只有平移关系 
    # 打印结果：
    ## [[ 1.      0.      0.      0.    ]
    ## [ 0.      1.      0.     -0.0106]
    ##  [ 0.      0.      1.      0.0144]
    ##  [ 0.      0.      0.      1.    ]]

    ## [[ 1.     0.     0.     0.   ]
    ## [ 0.     1.     0.    -0.016]
    ## [ 0.     0.     1.     0.013]
    ## [ 0.     0.     0.     1.   ]]
    # print ("transformation matrix bracket: ", trans_bracket_body) 
    # print ("transformation matrix head: ", trans_head_bracket)

    com = (homo_com_bracket[:3] * mass1+ homo_com_head[:3] * mass2)/total_mass
    # print("head com: ",com)
    return com, total_mass
    
# 脊椎简化为一条有质量的直线 加上一个servo motor和一个hip
def get_CoMofSpine(model, translation, spine, rotation_parent_world, length, length_yz):
    body_id_t1 = model.body_name2id('t1')
    body_id_t2 = model.body_name2id('t2')
    body_id_t3 = model.body_name2id('t3')
    body_id_t4 = model.body_name2id('t4')
    body_id_hip = model.body_name2id('hip')
    body_id_servo_spine = model.body_name2id('servo_spine')
    
    # 获取各个部件的质量
    mass_t1 = model.body_mass[body_id_t1]
    mass_t2 = model.body_mass[body_id_t2]
    mass_t3 = model.body_mass[body_id_t3]
    mass_t4 = model.body_mass[body_id_t4]

    mass_hip = model.body_mass[body_id_hip]
    mass_servo_spine = model.body_mass[body_id_servo_spine]
    mass_spine = mass_t1 + mass_t2 + mass_t3 + mass_t4 
    total_mass = mass_hip + mass_spine + mass_servo_spine #t1-t4 + hip + servo spine 质量
    # homo_com_body = np.append(com_parent, 1)
    trans_body_world = transformation_mat.get_homogeneous_transformation(rotation_parent_world, translation)
    trans_start_body = transformation_mat.get_transfomation_mat("t1") #spine的端点相对于parent
    homo_start_world = (trans_body_world @ trans_start_body)[:3,3]  #弧线起始点全局坐标
    start_world = homo_start_world[:3] #
    print("transformation t1: ", trans_start_body)  
    # print result
    # [[1.    0.    0.    0.   ]
    # [0.    1.    0.    0.064]
    # [0.    0.    1.    0.026]
    # [0.    0.    0.    1.   ]]

    # print("起始端点： ",start_world)
    # print("圆心角： ",theta)
    # print ("spine: ",spine)

    #获得spine末端相对于起始端点的变换矩阵
    trans_end_start = transformation_mat.get_trans_spine_start_to_end(spine, length, length_yz)
    #计算末端点的坐标系在世界坐标系的变换 world -> body -> start
    trans_start_world = trans_body_world @ trans_start_body
    trans_end_world = trans_body_world @ trans_start_body @ trans_end_start 
    theta = np.abs(2 * spine) #圆心角和偏转角度的关系 （弧度）
    if np.abs(theta) < 1e-7:  # 避免 theta 过小导致除零错误
        homo_end_pos = trans_end_world[:3,3]
        end_world = homo_end_pos[:3]
        com_spine = (start_world + end_world) / 2
    else:
        Radius = length / theta #标量
        x_dash = Radius * np.sin(spine) / spine#圆心到com的距离标量
        #计算com在起始端点坐标系的相对坐标
        if (spine > 0):
            translation_com_start = np.array([(Radius - x_dash * np.cos(spine)),  np.abs(x_dash * np.sin(spine)) , length_yz/2]) #-0.00125
        else:
            translation_com_start = np.array([-(Radius - x_dash * np.cos(spine)),  np.abs(x_dash * np.sin(spine)) , length_yz/2])

        #得到末端点的全局坐标
        homo_end_pos = trans_end_world[:3,3]
        end_world = homo_end_pos[:3]
        #spine的全局com
        homo_com_start = transformation_mat.get_homo_translation(translation_com_start)
        homo_com_spine = trans_start_world @ homo_com_start
        com_spine = homo_com_spine[:3]
        # print ("spine com: ", com_spine)
        print ("spine: ", spine)
        # print ("Radius: ", Radius)
        # print ("COM 在 start的位置： ",translation_com_start)

    # print("transformation matrix from start to end: ", trans_end_start)
    # hip的坐标系由end坐标系绕x轴旋转8.8度
    R_x_hip_end = R.from_euler('x', np.radians(8.8)).as_matrix()  # 绕 X 轴旋转 8.8°
    p_hip_end = np.array([0.0, 0.0165, -0.0009])  # hip 原点在 end 坐标系中的位置
    # 写出hip在end中的转移矩阵
    trans_hip_end = np.eye(4)
    trans_hip_end[:3, :3] = R_x_hip_end
    trans_hip_end[:3, 3] = p_hip_end
    # 写出T_hip_world
    trans_hip_world = trans_end_world @ trans_hip_end 
    # print ("transformation hip in world: ", trans_hip_world)
    
    homo_pos_hip_end = np.array([0.0, 0.0, 0.0, 1]) #相对于末端的坐标
    homo_com_hip = trans_hip_world @ homo_pos_hip_end
    com_hip = homo_com_hip[:3]
    # print ("com of hip: ", com_hip)
    # print ("com of the end: ", end_pos)
    homo_com_servo_spine = trans_hip_world @ transformation_mat.get_transfomation_mat("servo_spine") @ np.array([0,0,0,1])
    com_servo_spine = homo_com_servo_spine[:3]

    com = (mass_spine * com_spine + mass_hip * com_hip + mass_servo_spine * com_servo_spine) / total_mass
    
    return com, total_mass, com_hip, trans_hip_world

#尾巴部分 简化为一条直线 + 两个质点
def get_CoMofTail(trans_hip_world, tail_angle):
    #main 部分计算 pos="0 0.0217 0.0028" euler="15 0 0"  相对于hip
    T_tail_main_hip = np.array([0, 0.0217, 0.0028]) # 1*3
    R_tail_main_hip = R.from_euler('x', np.radians(-4)).as_matrix()  # 绕 X 轴旋转 8.8°
    trans_tail_main_hip = transformation_mat.get_homogeneous_transformation(R_tail_main_hip, T_tail_main_hip)
    trans_tail_main_world = trans_hip_world @ trans_tail_main_hip
    homo_com_tail_main = trans_tail_main_world[:3, 3]
    com_tail_main = homo_com_tail_main[:3]

    ##尾巴的计算
    trans_start_main = transformation_mat.get_translation_matrix(np.array([0, 0.03525, -0.0174]))
    translation_mat_com_start = transformation_mat.get_translation_matrix(np.array([0, 0.008*18/2, -0.0259/2]))
    homo_tail_com = (trans_tail_main_world @ trans_start_main @ translation_mat_com_start)[:3,3] # From tail.xml
    # print ("transformation matrix main to hip: ", trans_tail_main_hip)
    # print ("transformation matrix start to main: ", trans_start_main)
    # print ("transformation matrix hip to world: ", trans_hip_world)
    # tail_start = homo_tail_start[:3]
    # tail_end = homo_tail_end[:3]
    com_tail = homo_tail_com[:3]

    ##电机的计算
    T_servo_main = np.array([0, 0.0152, 0.001])
    R_servo_main = R.from_euler('x', np.radians(-19)).as_matrix()
    trans_servo_main = transformation_mat.get_homogeneous_transformation(R_servo_main, T_servo_main)
    trans_servo_world = trans_tail_main_world @ trans_servo_main
    homo_com_servo = trans_servo_world @ np.array([0,0,0,1])
    com_servo = homo_com_servo[:3]

    mass_servo_tail = 0.002
    mass_tail_main = 0.009
    mass_tail = 0.0095
    total_mass = mass_servo_tail + mass_tail + mass_tail_main



    com = (com_tail * mass_tail + com_servo * mass_servo_tail + com_tail_main * mass_tail_main) / total_mass
    return com, total_mass

##TODO
def get_com_fleg(angle_up, angle_down ,body_pos,leg_ID):
    ##通过当前腿部相位来确定腿的com
    return 0

def get_com_rleg(angle_up, angle_down ,body_pos,leg_ID):
    return 0


def get_CoM(model,sim,spine,length,length_yz):
    ##这里从mojuco获得仅供参考
    leg_id_rl = model.body_name2id('rl')
    leg_id_rr = model.body_name2id('rr')
    leg_id_fl = model.body_name2id('fl')
    leg_id_fr = model.body_name2id('fr')
    mass_legs = 0.00 # 取零则忽略腿部

    com_rl = sim.data.subtree_com[leg_id_rl]
    com_rr = sim.data.subtree_com[leg_id_rr]
    com_fl = sim.data.subtree_com[leg_id_fl]
    com_fr = sim.data.subtree_com[leg_id_fr]
    total_mass_legs = 4 * mass_legs
    # print ("leg mass: ",mass_legs)
    com_x = (com_rl[0] + com_rr[0] + com_fl[0] + com_fr[0]) / 4
    com_y = (com_rl[1] + com_rr[1] + com_fl[1] + com_fr[1]) / 4
    com_z = (com_rl[2] + com_rr[2] + com_fl[2] + com_fr[2]) / 4
    com_legs  = np.array([com_x,com_y,com_z])
    ##仅供参考

    global_com_body,body_mass,rotation_body_world, translation = get_CoMofBody(model,sim)
    global_com_head, head_mass= get_CoMofHead(model, translation, rotation_body_world)
    global_com_spine, spine_mass, global_com_hip, trans_hip_world = get_CoMofSpine(model, translation,spine,rotation_body_world,length,length_yz)
    global_com_tail, tail_mass = get_CoMofTail(trans_hip_world, 0)
    head_mass,spine_mass,tail_mass = head_mass, spine_mass, tail_mass
    
    total_mass = body_mass + head_mass + spine_mass + tail_mass
    
    com_robot = (
        global_com_body * body_mass + global_com_head * head_mass + global_com_spine * spine_mass + global_com_tail * tail_mass
        ) / total_mass
    # print ("总重量没算腿：", total_mass )
    # print ("global com of BODY: ", global_com_body)
    # print ("global com of HEAD: ", global_com_head)
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
        # com_vel = sim.data.subtree_linvel[body_id]
    else:
        total_com = sim.data.subtree_com[model.body_name2id('mouse')]
    
    return total_com

