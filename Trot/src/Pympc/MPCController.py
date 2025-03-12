import numpy as np
import osqp
import scipy.sparse as sparse
import matplotlib.pyplot as plt

class QuadrupedMPC:
    def __init__(self, N=10, dt=0.05):
        """
        N: 预测窗口（Prediction Horizon）
        dt: 时间步长（Timestep）
        """
        self.N = N
        self.dt = dt
        
        self.nx = 12  # 状态变量数 [x, y, z, roll, pitch, yaw, vx, vy, vz, omega_x, omega_y, omega_z]
        self.nu = 4   # 控制变量数 [fx, fy, fz, tau]
        
        self.Q = sparse.diags([1.0] * self.nx)  # 状态误差权重
        self.R = sparse.diags([0.1] * self.nu)  # 控制输入权重
        
    def setup_qp(self, x_ref, x_init):
        """ 构建 QP 问题 """
        H = np.kron(np.eye(self.horizon), self.Q)  # 状态误差权重矩阵
        G = np.kron(np.eye(self.horizon), self.R)  # 控制输入权重矩阵

        f = np.zeros(self.horizon * self.nx)  # 线性项
        A = np.zeros((self.horizon * self.nx, self.horizon * self.nu))  # 约束矩阵
        b = np.zeros(self.horizon * self.nx)  # 约束值

        return qpSWIFT.Problem(H, f, A, b, G)
    
    def solve(self, x_ref, x_init):
        qp = self.setup_qp(x_ref, x_init)
        u_opt = qp.solve()
        return u_opt[:self.nu]
    