#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于写示教函数，后期可能会加入一定的学习
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2019.5.13

import numpy as np
import numpy.linalg as nla
import math

import Kinematics as kin
import BaseFunction as bf
import FileOpen
import MachineLearning as ml
import RobotParameter as rp

#========================示教学习中使用的基本函数=======================#
#DMPS算法求解阶段,采用双曲线性变换
def dmps_solve_1(tau,k,d,g,x0,s,T,f,x_array,m_array):
    '''
    :param tau:
    :param k:
    :param d:
    :param alpha:
    :param g:
    :param x0:
    :param f:
    :return:
    '''
    #DMPS中间参数
    a = d/tau
    b = k/(tau**2)
    m = (k*g + k*(g - x0)*s + k*f)/(tau**2)

    #离散系统中间参数
    a1 = 4+ 2*T + T**2*b
    a2 = 2*T**2*b - 8
    a3 = 4 - 2*T + T**2*b

    #求取离散
    x = T**2/a3*(m_array[2] + m_array[1] +m_array[0]) - \
        a1/a3*x_array[1] - a2/a3*x_array[0]

    #更新数值
    xx_new = np.zeros(2)
    xx_new[0] = x_array[1]
    xx_new[1] = x

    mm_new = np.zeros(3)
    mm_new[0] = m_array[1]
    mm_new[1] = m_array[1]
    mm_new[2] = m
    return [xx_new, mm_new]

#采用迭代法求解
def dmps_solve_2(tau,k,d,g,x0,s,T,f,x_k,x_dot_k):
    '''
    :param tau:
    :param k:
    :param d:
    :param alpha:
    :param g:
    :param x0:
    :param f:
    :return:
    '''
    #上一时刻值
    xx_k = np.copy(x_k)
    xv_k = tau*x_dot_k

    #计算当前加速度
    iter_max= 50
    iter_time = 0
    x = 0
    x_d = 0

    while (True):
        x_dd = (k*(g-xx_k) - d*xv_k + k*(g - x0)*s + k*f)/(tau**2)
        x_d =x_dot_k + x_dd*T
        x = xx_k + x_d*T + x_dd*(T**2)/2
        if (abs(xx_k - x) < math.pow(10,-6) or iter_time > iter_max):
            break
        iter_time = iter_time + 1
        xx_k = x
        xv_k = tau * x_d
    return [x, x_d]

#========================示教阶段,获取关节角度,依次写入文件======================#
class teaching(object):
    def __init__(self, path):
        self.data_path = path
        self.qq_list = []

    #读取关节角
    def get_joint_position(self,qq):
        self.current_qq = qq
        self.qq_list.append(qq)

    #使用添加模式写入数据到指定文件
    def write_data(self):
        with open(self.data_path, 'w') as file_to_write:
            print ('开始读写')
            write_data = np.array(self.qq_list)
            [row, col] = write_data.shape
            for i in range(row):
                for j in range(col):
                    data = write_data[i, j]
                    file_to_write.write(str(data))
                    file_to_write.write(' ')  # 用空格分隔数据
                file_to_write.write('\n')  # 末尾转行符

#===========================学习阶段================================#
#示教学习函数类
class TeachingLearn(object):
    #类属性
    f_demo = np.zeros(1) #获取变量,方法中会动态修改形状

    def __init__(self, n_dof, dmps_path, rbf_path):
        #学习数据的维数
        self.n = n_dof
        self.dmps_path = dmps_path
        self.rbf_path = rbf_path

    #获取机器人参数
    def get_robot_paramter(self, DH_0):
        self.DH_0 = DH_0

    def get_dmps_paramter(self, tau, k, d):
        #时间尺度
        self.tau = tau #所有变量时间尺度同时，保证运动统一性
        #刚度项
        self.k = np.array(k)
        #阻尼项
        self.d = np.array(d)

        # 存储dmps的模型参数
        dmps_param = np.zeros([self.n, 3])
        dmps_param[0, 0] = self.tau #存储在第一位，其他位用0补齐
        dmps_param[:, 1] = self.k
        dmps_param[:, 2] = self.d
        self.dmps_param = dmps_param

    #高斯径向基参数
    def get_rbf_paramter(self, alpha, h):
        #rbf隐藏节点个数
        self.h = h

        # 正则系统常数,可以看做输入层处理函数
        self.alpha = alpha  # 正则系统时间常数所有自由度相同

    #示教函数
    def get_teaching_data(self, qq_list, g, X0, T):
        #关节角度
        self.qq_array = np.array(qq_list)
        #数据长度
        self.num = len(self.qq_array[:, 0])
        #时间和周期
        self.T = T
        self.tt = np.linspace(0, T*(self.num - 1), self.num)
        #正则时间
        self.ss = np.exp(-(self.alpha/self.tau)*self.tt)
        #初始位置
        self.X0 = X0
        #目标点位置
        self.goal = g

        #数据预处理
        self.data_preprocess()

    #数据处理函数
    def data_preprocess(self):
        #正运动学求取末端位姿
        X = np.zeros([self.num, 6])
        for i in range(self.num):
            X[i, :] = kin.fkine_euler(self.DH_0, self.qq_array[i, :])
        #求取末端位置的三次样条拟合
        X_xva = np.zeros([self.num, self.n, 3])
        for i in range(self.n):
            [s, vel, acc] = bf.spline1(self.tt, X[:, i], self.T, 0, 0)
            X_xva[:, i, 0] = s
            X_xva[:, i, 1] = vel
            X_xva[:, i, 2] = acc

        self.X_xva = X_xva

    #采用DMPS模型计算估计,高斯径向基函数拟合强迫项
    def learn(self):
        #计算DMPS模型计算强迫项
        f_demo = np.zeros([self.num, self.n])
        for i in range(self.n):
            f_demo[:, i] = (self.tau * self.X_xva[:, i, 2] + self.d[i] * self.X_xva[:, i, 1]) \
                          / self.k[i] - (self.goal[i] * np.ones(self.num) - self.X_xva[:, i, 0])\
                          - (self.goal[i] - self.X0[i]) * self.ss
        self.f_demo = f_demo

        #采用rbf拟合强迫项,求取rbf参数
        # 中心值,h*m个,时间变量为m=1
        c = np.linspace(self.ss[0], self.ss[-1], self.h)
        # 方差
        sigma = abs(self.ss[0] - self.ss[-1])
        #rbf隐藏层到输出层权重n*h
        w = ml.rbf_weight_oput_nout(self.ss, c, sigma, f_demo)

        # 存储rbf参数
        rbf_param = np.zeros([self.n + 2, self.h])
        rbf_param[0, 0] = self.alpha    #第一行存储时间正则常数
        rbf_param[0, 1] = sigma         #存在第二位,方差sigma
        rbf_param[1, :] = c             #第二行存放中心值
        rbf_param[2:self.n + 2, :] = w          #第三行后存储权重
        self.rbf_param = rbf_param

    #写入数据到指定文件
    def write_data(self):
        #rbf参数
        FileOpen.write(self.rbf_param, self.rbf_path)

        #写入dmps文件
        FileOpen.write(self.dmps_param, self.dmps_path)


#================================轨迹泛化,在现阶段==============================#
class TeachingReproduction(object):
    # 类属性
    f = np.zeros(1)  # 获取变量,方法中会动态修改形状
    def __init__(self, dmps_path, rbf_path):
        #参数路径
        self.dmps_path = dmps_path
        self.rbf_path = rbf_path
        #加载参数
        self.read_paramter()

    def read_paramter(self):
        #加载rbf参数
        rbf_param = FileOpen.read(self.rbf_path)
        # 正则系统常数
        self.alpha = rbf_param[0, 0]
        #方差sigma
        self.sigma = rbf_param[0, 1]
        #中心值
        self.c = rbf_param[1, :]
        #权重w
        self.w = rbf_param[2:, :]
        #输出自由度
        self.n = len(self.w[:, 0])

        #加载dmps参数
        dmps_param = FileOpen.read(self.dmps_path)
        #时间尺度
        self.tau = dmps_param[0, 0]
        # 刚度项
        self.k = dmps_param[:, 1]
        # 阻尼项
        self.d = dmps_param[:, 2]

    def reproduction(self, xx0, gg, tt, T):
        '''
        :param xx0: 规划起点
        :param gg: 规划目标点
        :param tt: 规划中采样时刻
        :return: 采样时刻对应的规划位置
        '''
        #规划时间
        ss = np.exp(-(self.alpha/self.tau)*tt)
        num = len(ss)

        #求取强迫项,用rbf求取强迫性
        f = np.zeros([num, self.n])
        for i in range(num):
            f[i, :] = ml.rbf_oput_nout(ss[i], self.c, self.sigma, self.w)
        self.f = f
        print f.shape

        #求取末端位置
        XX  = np.zeros([num, self.n])

        x_dot = np.zeros(self.n)
        xx = np.copy(xx0)

       #采用迭代发求微分方程
        for i in range(num):
            for j in range(self.n):
                [xx[j], x_dot[j]] = dmps_solve_2(
                    self.tau, self.k[j], self.d[j],
                    gg[j], xx0[j], ss[i], T, f[i, j],
                    xx[j], x_dot[j])
                XX[i, j] = xx[j]
        return XX













