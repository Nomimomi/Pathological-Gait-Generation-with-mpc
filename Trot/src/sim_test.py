from ToSim import SimModel
from Controller import MouseController
import matplotlib.pyplot as plt
import center_of_mass
import pandas as pd
import numpy as np
import argparse
from utils import utils
# --------------------
RUN_TIME_LENGTH = 4
if __name__ == '__main__':
	fre = 0.8
	delay_time = 0.2
	time_step = 0.002
	spine_angle = 15
	parser = argparse.ArgumentParser(description="Run simulation with different gaits.")
	# parser.add_argument('-walking', action='store_true', help='Use Walking gait')
	parser.add_argument('--gait', type=str, choices=['trot', 'walking', 'turningright', 'bounding'], default='trot', help='Choose gait type')
	args = parser.parse_args()
	gait_type = args.gait
	print(f"Selected gait type: {gait_type}")

	# 计算xy平面静止状态下的spine长度和yz平面的长度
	length_factor = 1
	length = length_factor * utils.calculate_spine_length_xy() 	#length of spine:  0.03361089834571633
	length_yz = length_factor * utils.calculate_spine_length_yz() #length of spine in z:  -0.014062572143769596

	run_steps_num = int(RUN_TIME_LENGTH / time_step) # 总共模拟步长
	delay_steps = int(delay_time / time_step)  

	render_flag = True
	theMouse = SimModel("../models/dynamic_4l.xml", render_flag)

	theController = MouseController(fre, time_step, spine_angle, gait_type)
	# theController = MouseMPCController(fre, time_step, spine_angle)

	period = theController.SteNum/2
	print("Step_num: ", period, run_steps_num)
	# 数据初始化
	curr_spine = 0
	initial_com = center_of_mass.get_CoM(theMouse.model,theMouse.sim, curr_spine,length,length_yz)

	print ("initial_com of Mass:", initial_com)
	log_data = []
	com_x_coords, com_y_coords, com_z_coords, steps= [], [], [], []
	com_x_eval, com_y_eval, com_z_eval = [], [], []

	is_motion_started = False
	ctrlData = [0.0, 1, 0.0, 1, 0.0, 1, 0.0, 1, 0,0,0,0]
	# for i in range(100):
	# 	ctrlData = [0.0, 1, 0.0, 1, 0.0, 1, 0.0, 1, 0,0,0,0]
	# 	theMouse.runStep(ctrlData, time_step, render_flag)
	# theMouse.initializing()
	
	period_x_list = []
	vel_x_list = []
	vel_y_list = []
	period_x = 0
	vel_x = 0
	period_t = []
	pre_com = initial_com

	difference_x, difference_y, difference_z = [], [], []

	for i in range(run_steps_num):
		# if i < delay_steps:
		# 	ctrlData = [0.0]*len(ctrlData)
		# else:
		tCtrlData = theController.runStep()
		ctrlData = tCtrlData
		theMouse.runStep(ctrlData, time_step, render_flag)
		# 打印每一步的com
		curr_com = center_of_mass.get_CoM_Reference(theMouse.model, theMouse.sim,1)
		com_vel = (curr_com - pre_com) / time_step
		vel_x_list.append(com_vel[0])
		vel_y_list.append(com_vel[1])
		pre_com = curr_com
		
		curr_spine = theController.getSpineVal(i)
		com_eval = center_of_mass.get_CoM(theMouse.model, theMouse.sim, curr_spine, length, length_yz)
		com_x_eval.append(com_eval[0])
		com_y_eval.append(com_eval[1])
		com_z_eval.append(com_eval[2])
		# print ("current spine angle: ", curr_spine)
		print(f"Step {i}: Center of  Mass {curr_com}")

		log_data.append({'Step': i, 'COM_X':curr_com[0], 'COM_Y':curr_com[1], 'COM_Z':curr_com[2]})
		com_x_coords.append(curr_com[0])
		com_y_coords.append(curr_com[1])
		com_z_coords.append(curr_com[2])
		steps.append(i)
		difference_x.append(com_eval[0]-curr_com[0])
		difference_y.append(com_eval[1]-curr_com[1])
		difference_z.append(com_eval[2]-curr_com[2])
		# print(f"Step {i}: differences {com_eval[0]-curr_com[0]}")
total_error = utils.calculate_error(run_steps_num, 500, com_x_coords, com_x_eval)
print("total error: ", total_error)
# 存储log数据-----------------------------------------------------
df = pd.DataFrame(log_data)
df.to_csv('center_of_mass_log.csv', index = False)
print("Log saved as 'center_of_mass_log.csv'.")

#region plot
plt.figure(figsize=(8,6))
plt.plot(steps, com_x_eval, label='evaluation', color='blue', linestyle='-', marker='o')  # 第一条线
plt.plot(steps, com_x_coords, label='reference', color='red', linestyle='-', marker='o')  # 第一条线
plt.plot(steps, difference_x, label = 'error', color = 'black', linestyle='--', marker='o')
plt.xlabel('Steps')
plt.ylabel('com in x-coordinates')
plt.title('com comparision')
plt.legend()  # 显示图例
plt.grid(True)  # 添加网格线（可选）

plt.figure(figsize=(8,6))
plt.plot(steps, com_y_eval, label='evaluation', color='blue', linestyle='-', marker='o')  # 第一条线
plt.plot(steps, com_y_coords, label='reference', color='red', linestyle='-', marker='o')  # 第一条线
plt.plot(steps, difference_y, label = 'error', color = 'black', linestyle='--', marker='o')
plt.xlabel('Steps')
plt.ylabel('com in y-coordinates')
plt.title('com comparision')
plt.legend()  # 显示图例
plt.grid(True)  # 添加网格线（可选）

plt.figure(figsize=(8,6))
plt.plot(steps, com_z_eval, label='evaluation', color='blue', linestyle='-', marker='o')  # 第一条线
plt.plot(steps, com_z_coords, label='reference', color='red', linestyle='-', marker='o')  # 第一条线
plt.plot(steps, difference_z, label = 'error', color = 'black', linestyle='--', marker='o')
plt.xlabel('Steps')
plt.ylabel('com in y-coordinates')
plt.title('com comparision')
plt.legend()  # 显示图例
plt.grid(True)  # 添加网格线（可选）

plt.show()
#endregion