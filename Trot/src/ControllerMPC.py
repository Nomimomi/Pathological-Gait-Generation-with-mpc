import numpy as np
import math
import casadi as ca
import qpSWIFT
from Controller import MouseController
from LegModel.forPath import LegPath
from LegModel.legs import LegModel

class ControllerMPC(MouseController):
    def __init__(self, dt=0.01, N=10, m=0.2515, leg_params = None):
        self.dt = dt  # 时间步长
        self.N = N    # horizon
        self.m = m    # 质量
        self.g = np.array([0, 0, -9.81])  # 重力

        # 状态维度 (12) 和输入维度 (12)
        self.nx = 12  # [px, py, pz, vx, vy, vz, phi, theta, psi, wx, wy, wz]
        self.nu = 8  # [dq11, dq12, dq21, dq22, dq31, dq32, dq41, dq42]
        
        # 动态矩阵（简化）
        self.A = np.block([
            [np.eye(3), self.dt * np.eye(3)],
            [np.zeros((3, 3)), np.eye(3)]
        ])

        self.B = np.zeros((self.nx, self.nu))
        self.B[3:6, :] = np.kron(np.ones((1, 4)), np.eye(3)) * self.dt / self.m  # 速度-力
        # 角速度-力（简化）
        self.G = np.zeros(self.nx)
        self.G[3:6] = self.g * self.dt  # 重力影响

        # 足端位置（相对于质心）
        self.r = np.array([[0.2, 0.1, -0.1], [0.2, -0.1, -0.1], 
                          [-0.2, 0.1, -0.1], [-0.2, -0.1, -0.1]])  # [x, y, z]

        # 权重矩阵
        self.Q = np.eye(self.nx) * 10  # 状态权重
        self.R = np.eye(self.nu) * 0.1  # 输入权重

        # 状态空间矩阵 

    def printParams(self):
        print ("spine angle: ",  self.spine_A)
        print ("Step Number: ", self.time_step)
        return 0

    def getLegCtrl(self, leg_M, curStep, leg_ID):
        curStep = curStep % self.SteNum
        return 0

# if __name__ == '__main__':
#     theController = ControllerMPC(fre = 0.8, time_step=0.002, spine_angle = 15)
#     theController.printParams()