import numpy as np
import math
import casadi as ca
import qpSWIFT

from LegModel.forPath import LegPath
from LegModel.legs import LegModel

class MouseController(object):
	"""docstring for MouseController"""
	def __init__(self, fre, time_step, spine_angle, gait_type = "trot"):
		super(MouseController, self).__init__()
		PI = np.pi
		self.curStep = 0
		
		self.turn_F = 6*PI/180
		self.turn_H = 3*PI/180
		self.pathStore = LegPath()
		self.turnrate = 1.5

		# ---------- Todo ---------- #
		# Define the phase difference of the gait pattern
		if gait_type == "trot":
			self.phaseDiff = [PI, 0, 3*PI/4, PI]
		elif gait_type == "walking":
			self.phaseDiff = [0, PI / 2, PI, 3 * PI / 2]
		else:
			raise ValueError(f"Unsupported gait type: {gait_type}")
		# self.phaseDiff = [PI + self.turnrate, 0 - self.turnrate, 0 - self.turnrate, PI + self.turnrate] # Turning Right
		# self.phaseDiff = [0, 0, PI, PI] # Bounding

		# ---------- Todo ---------- #

		self.period = 2/2
		self.fre_cyc = fre
		self.SteNum = int(1/(time_step*self.fre_cyc))
		print("Step Num. ----> ", self.SteNum)
		self.spinePhase = self.phaseDiff[3]
		# --------------------------------------------------------------------- #
		self.spine_A =2*spine_angle
		print("angle --> ", spine_angle)
		self.spine_A = self.spine_A*PI/180 #spine的活动区间转换为弧度形式
		# --------------------------------------------------------------------- #
		leg_params = [0.031, 0.0128, 0.0118, 0.040, 0.015, 0.035]
		self.fl_left = LegModel(leg_params)
		self.fl_right = LegModel(leg_params)
		self.hl_left = LegModel(leg_params)
		self.hl_right = LegModel(leg_params)
		# --------------------------------------------------------------------- #
		self.stepDiff = [0,0,0,0]
		for i in range(4):
			self.stepDiff[i] = int(self.SteNum * self.phaseDiff[i]/(2*PI))
		self.stepDiff.append(int(self.SteNum * self.spinePhase/(2*PI)))


	def getLegCtrl(self, leg_M, curStep, leg_ID):
		curStep = curStep % self.SteNum
		turnAngle = self.turn_F
		leg_flag = "F"
		if leg_ID > 1:
			leg_flag = "H"
			turnAngle = self.turn_H
		radian = 2*np.pi * curStep/self.SteNum
		currentPos = self.pathStore.getOvalPathPoint(radian, leg_flag, self.period)
		trg_x = currentPos[0]
		trg_y = currentPos[1]

		tX = math.cos(turnAngle)*trg_x - math.sin(turnAngle)*trg_y
		tY = math.cos(turnAngle)*trg_y + math.sin(turnAngle)*trg_x
		qVal = leg_M.pos_2_angle(tX, tY)
		return qVal	

	def getSpineVal(self, spineStep):
		# ---------- Todo ---------- #
		radian = 2 * np.pi * spineStep / self.SteNum
		spineAngle = self.spine_A * np.sin(radian)
		return spineAngle		#输出弧度形式
		 

	def runStep(self):
		foreLeg_left_q = self.getLegCtrl(self.fl_left, 
			self.curStep + self.stepDiff[0], 0)
		foreLeg_right_q = self.getLegCtrl(self.fl_right, 
			self.curStep + self.stepDiff[1], 1)
		hindLeg_left_q = self.getLegCtrl(self.hl_left, 
			self.curStep + self.stepDiff[2], 2)
		hindLeg_right_q = self.getLegCtrl(self.hl_right, 
			self.curStep + self.stepDiff[3], 3)

		# ---------- Todo ---------- #
		spineStep = self.curStep + self.stepDiff[1]
		# ---------- Todo ---------- #
		spine = self.getSpineVal(spineStep)
		# print ("current spine: " ,spine)
		# spine = 0
		self.curStep = (self.curStep + 1) % self.SteNum

		ctrlData = []
		ctrlData.extend([0,0]) # 
		ctrlData.extend(foreLeg_right_q)
		ctrlData.extend(hindLeg_left_q)
		ctrlData.extend(hindLeg_right_q)
		for i in range(3):
			ctrlData.append(0) ##tail neck head
		ctrlData.append(spine)
		return ctrlData
