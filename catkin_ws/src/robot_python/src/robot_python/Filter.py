#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于书写滤波器算法
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

#====有限脉冲响应滤波器=======#
class FIRFilter(object):
    def __init__(self, wc=0.0314, N=20):
        # 截断频率
        self.wc = wc
        # 输入阶数
        self.N = N

    def set_input_init(self, x0):
        #获取变量维数
        self.m = len(x0)
        self.x0 = list(x0)

    def set_rectangle_filter(self):
        # 计算矩形窗加窗后脉冲响应函数
        self.h_r = np.zeros(self.N)
        alpha = (self.N-1)/2.0
        for i in range(self.N):
            if(abs(i - alpha)<pow(10, -6)):
                self.h_r[i] = self.wc/np.pi
            else:
                self.h_r[i] = np.sin(self.wc*(i - alpha))/(np.pi*(i-alpha))
        self.h_r = self.h_r/np.sum(self.h_r)
        # 设计输入初值
        self.X_r = list(np.zeros([self.N, self.m]))
        for i in range(self.N):
            self.X_r[i] = self.x0

    def set_hanning_filter(self):
        # 计算汉宁加窗后脉冲响应函数
        self.h_n = np.zeros(self.N)
        alpha = (self.N - 1) / 2.0
        for i in range(self.N):
            if (abs(i - alpha) < pow(10, -6)):
                self.h_n[i] = 0.5*(1-np.cos(2*i*np.pi/(self.N-1)))*self.wc / np.pi
            else:
                self.h_n[i] = 0.5*(1-np.cos(2 * i * np.pi/(self.N-1))) * \
                              np.sin(self.wc * (i - alpha)) / (np.pi * (i - alpha))

        self.h_n = self.h_n / np.sum(self.h_n)
        # 设计输入初值
        self.X_n = list(np.zeros([self.N, self.m]))
        for i in range(self.N):
            self.X_n[i] = self.x0

    #矩形窗滤波器
    def rectangle_filter(self, x):
        #更新输入
        self.X_r.append(x)
        del self.X_r[0]

        #计算输出
        X = np.array(self.X_r)
        y = np.dot(X.T, self.h_r)
        return y

    #汉宁窗滤波器
    def hanning_filter(self, x):
        # 更新输入
        self.X_n.append(x)
        del self.X_n[0]

        # 计算输出
        X = np.array(self.X_n)
        y = np.dot(X.T, self.h_n)
        return y

class FIRFilter_sigle(object):
    def __init__(self, wc=0.0314, N=20):
        # 截断频率
        self.wc = wc
        # 输入阶数
        self.N = N

    def set_input_init(self, x0):
        #获取变量维数
        self.x0 = x0

    def set_rectangle_filter(self):
        # 计算矩形窗加窗后脉冲响应函数
        self.h_r = np.zeros(self.N)
        alpha = (self.N-1)/2.0
        for i in range(self.N):
            if(abs(i - alpha)<pow(10, -6)):
                self.h_r[i] = self.wc/np.pi
            else:
                self.h_r[i] = np.sin(self.wc*(i - alpha))/(np.pi*(i-alpha))
        self.h_r = self.h_r/np.sum(self.h_r)
        # 设计输入初值
        self.X_r = list(np.zeros(self.N))
        for i in range(self.N):
            self.X_r[i] = self.x0

    def set_hanning_filter(self):
        # 计算汉宁加窗后脉冲响应函数
        self.h_n = np.zeros(self.N)
        alpha = (self.N - 1) / 2.0
        for i in range(self.N):
            if (abs(i - alpha) < pow(10, -6)):
                self.h_n[i] = 0.5*(1-np.cos(2*i*np.pi/(self.N-1)))*self.wc / np.pi
            else:
                self.h_n[i] = 0.5*(1-np.cos(2 * i * np.pi/(self.N-1))) * \
                              np.sin(self.wc * (i - alpha)) / (np.pi * (i - alpha))

        self.h_n = self.h_n / np.sum(self.h_n)
        # 设计输入初值
        self.X_n = list(np.zeros(self.N))
        for i in range(self.N):
            self.X_n[i] = self.x0

    #矩形窗滤波器
    def rectangle_filter(self, x):
        #更新输入
        self.X_r.append(x)
        del self.X_r[0]

        #计算输出
        X = np.array(self.X_r)
        y = np.dot(X.T, self.h_r)
        return y

    #汉宁窗滤波器
    def hanning_filter(self, x):
        # 更新输入
        self.X_n.append(x)
        del self.X_n[0]

        # 计算输出
        X = np.array(self.X_n)
        y = np.dot(X.T, self.h_n)
        return y

    # 矩形窗滤波器
    def rectangle_array_filter(self, x):
        m = len(x)
        y = np.zeros(m)
        for i in range(m):
            # 更新输入
            self.X_r.append(x[i])
            del self.X_r[0]

            # 计算输出
            X = np.array(self.X_r)
            y[i] = np.dot(X.T, self.h_r)
        return y

    # 汉宁窗滤波器
    def hanning_array_filter(self, x):
        m = len(x)
        y = np.zeros(m)
        for i in range(m):
            # 更新输入
            self.X_n.append(x[i])
            del self.X_n[0]

            # 计算输出
            X = np.array(self.X_n)
            y[i] = np.dot(X.T, self.h_n)
        return y



def FIR_filter():
    #从上文有限冲击响应滤波器建立六维力滤波器
    wc = np.pi/10.0
    N = 20
    FIR_filt = FIRFilter(wc, N)
    F0 = np.full(6, 10)
    FIR_filt.set_input_init(F0)
    FIR_filt.set_hanning_filter()
    FIR_filt.set_rectangle_filter()

    #获取滤波后的数据：本文制造一组虚拟数据
    n = 6
    num = 1000
    T = 0.01
    t = np.linspace(0, (num - 1)*T, num)
    F = np.zeros([num, n])
    for i in range(n):
        F[:, i] = 10 + 1*np.sin(np.pi/2*t) + 1*np.random.randn(num)

    #带入滤波器
    Fr_filt = np.zeros([num, n])
    Fn_filt = np.zeros([num, n])
    for i in range(num):
        Fr_filt[i, :] = FIR_filt.rectangle_filter(F[i, :])
        Fn_filt[i, :] = FIR_filt.hanning_filter(F[i, :])

    #绘画函数,仅考虑第一维
    plt.figure(1)
    plt.plot(t, F[:, 0], label='Fx', color='b')
    plt.plot(t, Fr_filt[:, 0], label='Fr_filt', color='r')
    plt.plot(t, Fn_filt[:, 0], label='Fn_filt', color='g')
    plt.title("FIR_filter")
    plt.xlabel("t/s")
    plt.ylabel("f/N")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    #wrench_filter()
    FIR_filter()
    print "finish!"