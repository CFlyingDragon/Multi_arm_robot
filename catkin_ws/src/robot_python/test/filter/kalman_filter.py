#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于书写卡尔曼滤波器算法
#程序员：陈永*
#版权：哈尔滨工业大学(深圳)
#日期：2020.10.4

import numpy as np
import matplotlib.pyplot as plt

#====建立全状态卡尔曼滤波方程=======#
class KalmanFilter(object):
    def __init__(self):
        pass

    #获取建立状态方程和观测方程的参数
    def get_state_measurement_matrix(self, A, B, C):
        #状态转移矩阵
        self.A = np.mat(A)
        #激励转移矩阵
        self.B = np.mat(B)
        #观测矩阵
        self.C = np.mat(C)

    #获取激励矢量和测量误差矢量的协方差
    def get_cov_matrix(self, R, Q = None):
        #误差协方差矩阵
        self.R = np.mat(R)
        #激励激励协方差矩阵
        self.Q = np.mat(Q)

    #获得转移状态协方差矩阵初值
    def get_state_cov_matrix(self, P0):
        self.P = np.mat(P0)

    #获取状态矢量初值
    def get_state_init(self, x0):
        self.x = np.array(x0)

    #计算卡尔曼增益
    def kalman_gain(self):
        #求取卡尔曼增益
        self.K = self.P*self.C.T*(self.C*self.P*self.C.T + self.R).I

        #更新状态转移协方差矩阵
        self.P = self.P - self.K*self.C*self.P

    #输出估计值
    def out_filter_value(self, z):
        # 计算卡尔曼增益
        self.kalman_gain()
        D = np.array(self.A - self.K*self.C*self.A)

        self.x = np.dot(D, self.x) + np.dot(np.array(self.K), z)
        return self.x

def wrench_filter():
    #从上文卡尔曼滤波建立六维力滤波器
    kalm_filt = KalmanFilter()

    #建立状态方程和观测方程:n=m=p
    n = 6
    I = np.eye(n)
    A = np.copy(I)
    B = 0.1*I
    C = np.copy(I)
    kalm_filt.get_state_measurement_matrix(A, B, C)

    #输入测量误差协方差方程
    R = 0.001*I
    kalm_filt.get_cov_matrix(R)

    #输入状态初值
    x0 = np.zeros(n)
    P0 = np.copy(I)
    kalm_filt.get_state_cov_matrix(P0)
    kalm_filt.get_state_init(x0)

    #获取滤波后的数据：本文制造一组虚拟数据
    num = 1000
    T = 0.01
    t = np.linspace(0, (num - 1)*T, num)
    F = np.zeros([num, n])
    for i in range(n):
        F[:, i] = 10 + 1*np.random.randn(num)

    #带入滤波器
    F_filt = np.zeros([num, n])
    for i in range(num):
        F_filt[i, :] = kalm_filt.out_filter_value(F[i, :])

    #绘画函数,仅考虑第一维
    plt.figure(1)
    plt.plot(t, F[:, 0], label='Fx', color='b')
    plt.plot(t, F_filt[:, 0], label='Fx_filt', color='r')
    plt.title("Kalman_filter")
    plt.xlabel("t/s")
    plt.ylabel("f/N")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    wrench_filter()