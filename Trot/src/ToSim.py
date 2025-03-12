from mujoco_py import load_model_from_path, MjSim, MjViewer
import matplotlib.pyplot as plt
import numpy as np
import math

import time
class SimModel(object):
	"""docstring for SimModel"""
	def __init__(self, modelPath, render_flag = False):
		super(SimModel, self).__init__()
		self.model = load_model_from_path(modelPath)
		self.sim = MjSim(self.model)
		if render_flag:
			self.viewer = MjViewer(self.sim)
			self.viewer.cam.azimuth = 0
			self.viewer.cam.lookat[0] += 0.25
			self.viewer.cam.lookat[1] += -0.5
			self.viewer.cam.distance = self.model.stat.extent * 0.5
			self.viewer.run_speed = 2

		self.sim_state = self.sim.get_state()
		self.sim.set_state(self.sim_state)
		
	def initializing(self):
		self.time_counter = 0

		
	def runStep(self, ctrlData, cur_time_step, render_flag):
		# ------------------------------------------ #
		# ID 0, 1 left-fore leg and coil 
		# ID 2, 3 right-fore leg and coil
		# ID 4, 5 left-hide leg and coil
		# ID 6, 7 right-hide leg and coil
		# Note: For leg, it has [-1: front; 1: back]
		# Note: For fore coil, it has [-1: leg up; 1: leg down]
		# Note: For hide coil, it has [-1: leg down; 1: leg up]
		# ------------------------------------------ #
		# ID 08 is neck		(Horizontal)
		# ID 09 is head		(vertical)
		# ID 10 is spine	(Horizontal)  [-1: right, 1: left]
		# Note: range is [-1, 1]
		# ------------------------------------------ #
		step_num = int(cur_time_step/0.002)
		self.sim.data.ctrl[:] = ctrlData
		for i in range(step_num):
			self.sim.step()
			if render_flag:
				self.viewer.render()

	def getTime(self):
		return self.sim.data.time
	
