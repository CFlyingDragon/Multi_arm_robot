#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于写示教函数，后期可能会加入一定的学习
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2019.5.13

import os
import numpy as np
import numpy.linalg as nla
import math

import Kinematics as kin
import BaseFunction as bf
import FileOpen
import MachineLearning as ml
import RobotParameter as rp
import MyPlot

#========================示教学习中使用的基本函数=======================#
#高斯径向基函数
def gauss_radial_basis_function(h, c, s):
    psi = math.exp(-h*(s - c)**2)
    return psi

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

    x_dd = (k*(g-xx_k) - d*xv_k + k*(g - x0)*s + k*f)/(tau**2)

    x_d =x_dot_k + x_dd*T
    x = xx_k + x_d*T

    return [x, x_d]

#========================示教阶段,获取关节角度,依次写入文件======================#
class teaching(object):
    def __init__(self, path):
        self.data_path = path
        self.qq_list = []

    #读取关节角
    def get_joint_position(self, qq):
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
#设置为已知dmps参数和
class TeachingLearn1(object):
    X_xva = np.zeros(6)
    def __init__(self, m_dof, dmps_path, rbf_path):
        # 学习数据的维数
        self.m = m_dof
        self.dmps_path = dmps_path
        self.rbf_path = rbf_path

        self.get_dmps_paramter()

    def get_dmps_paramter(self):
        # 加载dmps参数
        dmps_param = FileOpen.read(self.dmps_path)
        # 时间尺度
        self.tau = dmps_param[0, 0]
        # 正则常数
        self.alpha = dmps_param[1, 0]
        # 刚度项
        self.k = dmps_param[:, 1]
        # 阻尼项
        self.d = dmps_param[:, 2]

    # 高斯径向基参数
    def get_rbf_paramter(self, h):
        # rbf隐藏节点个数
        self.h = h

    # 示教函数
    def get_teaching_data(self, xx, xv, xa, g, X0, T):
        #示教数据
        # 数据长度
        self.num = len(xx)
        self.X_xva = np.zeros([self.num, self.m, 3])
        self.X_xva[:, :, 0] = np.array(xx)
        self.X_xva[:, :, 1] = np.array(xv)
        self.X_xva[:, :, 2] = np.array(xa)

        # 时间和周期
        self.T = T
        self.tt = np.linspace(0, T * (self.num - 1), self.num)
        # 正则时间
        self.ss = np.exp(-(self.alpha / self.tau) * self.tt)
        # 初始位置
        self.X0 = X0
        # 目标点位置
        self.goal = g

    # 采用DMPS模型计算估计,高斯径向基函数拟合强迫项
    def learn(self):
        # 计算DMPS模型计算强迫项
        f_demo = np.zeros([self.num, self.m])
        for i in range(self.m):
            f_demo[:, i] = (self.tau * self.tau * self.X_xva[:, i, 2] + self.d[i] * self.X_xva[:, i, 1]) \
                           / self.k[i] - (self.goal[i] * np.ones(self.num) - self.X_xva[:, i, 0]) \
                           - (self.goal[i] - self.X0[i]) * self.ss
        self.f_demo = f_demo

        # 采用rbf拟合强迫项,求取rbf参数
        # 中心值,h*m个,时间变量为m=1
        c = np.linspace(self.ss[0], self.ss[-1], self.h)
        # 方差
        sigma = abs(self.ss[0] - self.ss[-1])
        # rbf隐藏层到输出层权重n*h
        w = ml.rbf_weight_oput_nout(self.ss, c, sigma, f_demo)

        # 存储rbf参数
        rbf_param = np.zeros([self.m + 2, self.h])
        rbf_param[0, 0] = sigma  # 存在第1位,方差sigma
        rbf_param[1, :] = c  # 第二行存放中心值
        rbf_param[2:self.m + 2, :] = w  # 第三行后存储权重
        self.rbf_param = rbf_param

    # 写入数据到指定文件
    def write_data(self):
        # rbf参数
        FileOpen.write(self.rbf_param, self.rbf_path)

# 设置为已知dmps参数和
class TeachingLearn(object):
    X_xva = np.zeros(6)

    def __init__(self, m_dof, dmps_path, rbf_path):
        # 学习数据的维数
        self.m = m_dof
        self.dmps_path = dmps_path
        self.rbf_path = rbf_path

        self.get_dmps_paramter()

    # 获取机器人参数
    def get_robot_paramter(self, DH_0):
        self.DH_0 = DH_0

        # 创建运动学
        self.kin1 = kin.GeneralKinematic(self.DH_0)

    def get_dmps_paramter(self):
        # 加载dmps参数
        dmps_param = FileOpen.read(self.dmps_path)
        # 时间尺度
        self.tau = dmps_param[0, 0]
        # 正则常数
        self.alpha = dmps_param[1, 0]
        # 刚度项
        self.k = dmps_param[:, 1]
        # 阻尼项
        self.d = dmps_param[:, 2]

    # 高斯径向基参数
    def get_rbf_paramter(self, h):
        # rbf隐藏节点个数
        self.h = h

    # 示教函数
    def get_teaching_data(self, qq_list, g, X0, T):
        # 关节角度
        self.qq_array = np.array(qq_list)
        # 数据长度
        self.num = len(self.qq_array[:, 0])
        # 时间和周期
        self.T = T
        self.tt = np.linspace(0, T * (self.num - 1), self.num)
        # 正则时间
        self.ss = np.exp(-(self.alpha / self.tau) * self.tt)
        # 初始位置
        self.X0 = X0
        # 目标点位置
        self.goal = g

        # 数据预处理
        self.data_preprocess()

    # 数据处理函数
    def data_preprocess(self):
        # 正运动学求取末端位姿
        X = np.zeros([self.num, 6])
        for i in range(self.num):
            X[i, :] = kin.fkine_euler(self.DH_0, self.qq_array[i, :])
        #求取末端位置的,前向差分获取速度和加速度
        X_xva = np.zeros([self.num, self.m, 3])
        for i in range(self.num):
            X_xva[i, :, 0] = X[i, :]
            if (i==0):
                X_xva[i, :, 1] = 0
                X_xva[i, :, 2] = 0
            else:
                X_xva[i, :, 1] = (X_xva[i, :, 0] - X_xva[i-1, :, 0])/self.T
                X_xva[i, :, 2] = (X_xva[i, :, 1] - X_xva[i-1, :, 1])/self.T

        MyPlot.plot2_nd(self.tt, X_xva[:, :, 0], title="X_xva")
        MyPlot.plot2_nd(self.tt, X_xva[:, :, 1], title="X_xva")
        MyPlot.plot2_nd(self.tt, X_xva[:, :, 2], title="X_xva")

        self.X_xva = X_xva

    # 采用DMPS模型计算估计,高斯径向基函数拟合强迫项
    def learn(self):
        # 计算DMPS模型计算强迫项
        f_demo = np.zeros([self.num, self.m])
        for i in range(self.m):
            f_demo[:, i] = (self.tau * self.tau * self.X_xva[:, i, 2] + self.d[i] * self.X_xva[:, i, 1]) \
                           / self.k[i] - (self.goal[i] * np.ones(self.num) - self.X_xva[:, i, 0]) \
                           - (self.goal[i] - self.X0[i]) * self.ss
        self.f_demo = f_demo

        # 采用rbf拟合强迫项,求取rbf参数
        # 中心值,h*m个,时间变量为m=1
        c = np.linspace(self.ss[0], self.ss[-1], self.h)
        # 方差
        sigma = abs(self.ss[0] - self.ss[-1])
        # rbf隐藏层到输出层权重n*h
        w = ml.rbf_weight_oput_nout(self.ss, c, sigma, f_demo)

        # 存储rbf参数
        rbf_param = np.zeros([self.m + 2, self.h])
        rbf_param[0, 0] = sigma  # 存在第1位,方差sigma
        rbf_param[1, :] = c  # 第二行存放中心值
        rbf_param[2:self.m + 2, :] = w  # 第三行后存储权重
        self.rbf_param = rbf_param

    # 写入数据到指定文件
    def write_data(self):
        # rbf参数
        FileOpen.write(self.rbf_param, self.rbf_path)

#================================轨迹泛化,在现阶段==============================#
class TeachingReproduction(object):
    def __init__(self, dmps_path, rbf_path):
        #参数路径
        self.dmps_path = dmps_path
        self.rbf_path = rbf_path
        #加载参数
        self.read_paramter()

    def read_paramter(self):
        #加载rbf参数
        rbf_param = FileOpen.read(self.rbf_path)

        #方差sigma
        self.sigma = rbf_param[0, 0]
        #中心值
        self.c = rbf_param[1, :]
        #权重w
        self.w = rbf_param[2:, :] #w= m*h
        #输出自由度
        self.m = len(self.w[:, 0])

        #加载dmps参数
        dmps_param = FileOpen.read(self.dmps_path)
        #时间尺度
        self.tau = dmps_param[0, 0]
        # 正则系统常数
        self.alpha = dmps_param[1, 0]
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
        f = np.zeros([num, self.m])
        for i in range(num):
            f[i, :] = ml.rbf_oput_nout(ss[i], self.c, self.sigma, self.w)
        self.f = f
        print f.shape

        #求取末端位置
        XX  = np.zeros([num, self.m])

        x_dot = np.zeros(self.m)
        xx = np.copy(xx0)

       #采用迭代发求微分方程
        for i in range(num):
            for j in range(self.m):
                [xx[j], x_dot[j]] = dmps_solve_2(
                    self.tau, self.k[j], self.d[j],
                    gg[j], xx0[j], ss[i], T, f[i, j],
                    xx[j], x_dot[j])
                XX[i, j] = xx[j]
        self.xx = np.copy(XX)

    def get_plan_tcp(self):
        return self.xx

    def get_plan_joint(self, DH_0):
        #建立运动学
        kin1 = kin.GeneralKinematic(DH_0)

# ================================采用局部加权回归算法拟合==============================#
#设置为已知dmps参数和
class TeachingLearnLocal(object):
    def __init__(self, m_dof, dmps_path, weight_path):
        # 学习数据的维数
        self.m = m_dof
        self.dmps_path = dmps_path
        self.weight_path = weight_path

        self.get_dmps_paramter()

    # 获取机器人参数
    def get_robot_paramter(self, DH_0):
        self.DH_0 = DH_0

        #创建运动学
        self.kin1 = kin.GeneralKinematic(self.DH_0)

    def get_dmps_paramter(self):
        # 加载dmps参数
        dmps_param = FileOpen.read(self.dmps_path)
        # 时间尺度
        self.tau = dmps_param[0, 0]
        # 正则系数
        self.alpha = dmps_param[1, 0]
        # 刚度项
        self.k = dmps_param[:, 1]
        # 阻尼项
        self.d = dmps_param[:, 2]

    # 高斯径向基参数
    def get_weight_num(self, w_num, h):
        # rbf隐藏节点个数
        self.w_num = w_num

        #带宽
        self.h = h

    # 示教函数
    def get_teaching_data(self, qq_list, g, X0, T):
        # 关节角度
        self.qq_array = np.array(qq_list)
        # 数据长度
        self.num = len(self.qq_array[:, 0])
        # 时间和周期
        self.T = T
        self.tt = np.linspace(0, T * (self.num - 1), self.num)
        # 正则时间
        self.ss = np.exp(-(self.alpha / self.tau) * self.tt)
        # 初始位置
        self.X0 = X0
        # 目标点位置
        self.goal = g

        # 数据预处理
        self.data_preprocess()

    # 数据处理函数
    def data_preprocess(self):
        # 正运动学求取末端位姿
        X = np.zeros([self.num, 6])
        for i in range(self.num):
            X[i, :] = kin.fkine_euler(self.DH_0, self.qq_array[i, :])
        # 求取末端位置的三次样条拟合
        X_xva = np.zeros([self.num, self.m, 3])
        for i in range(self.m):
            [s, vel, acc] = bf.spline1(self.tt, X[:, i], self.T, 0, 0)
            X_xva[:, i, 0] = s
            X_xva[:, i, 1] = vel
            X_xva[:, i, 2] = acc

        self.X_xva = X_xva

    # 采用DMPS模型计算估计,高斯径向基函数拟合强迫项
    def learn(self):
        # 计算DMPS模型计算强迫项
        f_demo = np.zeros([self.num, self.m])
        for i in range(self.m):
            f_demo[:, i] = (self.tau * self.X_xva[:, i, 2] + self.d[i] * self.X_xva[:, i, 1]) \
                           / self.k[i] - (self.goal[i] * np.ones(self.num) - self.X_xva[:, i, 0]) \
                           - (self.goal[i] - self.X0[i]) * self.ss
        self.f_demo = f_demo

        #选取中心值
        c = np.linspace(self.ss[0], self.ss[-1], self.w_num)
        self.c = c

        #求取径向基矩阵
        psi = np.zeros([self.m, self.num, self.w_num])
        for i in range(self.m):
            for j in range(self.num):
                for k in range(self.w_num):
                    psi[i, j, k] = gauss_radial_basis_function(self.h, c[k], self.ss[j])

        #求取中间矩阵
        T = np.zeros([self.m, self.num, self.w_num])
        for i in range(self.m):
            for j in range(self.num):
                psi_sum = np.sum(psi[i, j, :])
                for k in range(self.w_num):
                    T[i, j, k] = psi[i, j, k]*self.ss[j]/psi_sum

        #权重矩阵
        w = np.zeros([self.w_num, self.m])
        for i in range(self.m):
            w[:, i] = np.dot(nla.pinv (T[i, :, :]), self.f_demo[:, i])

        # 存储rbf参数
        self.weight_param = np.zeros([self.w_num, self.m + 2])
        self.weight_param[0, 0] = self.h
        self.weight_param[:, 1] = self.c
        self.weight_param[:, 2:] = w

    # 写入数据到指定文件
    def write_data(self):
        # rbf参数
        FileOpen.write(self.weight_param, self.weight_path)

#================================轨迹泛化,在现阶段==============================#
class TeachingReproductionLocal(object):
    def __init__(self, dmps_path, weight_path):
        #参数路径
        self.dmps_path = dmps_path
        self.weight_path = weight_path
        #加载参数
        self.read_paramter()

    def read_paramter(self):
        #加载rbf参数
        local_param = FileOpen.read(self.weight_path)
        # 正则系统常数
        self.h = local_param[0, 0]
        #中心值
        self.c = local_param[:, 1]
        #权重w
        self.w = local_param[:, 2:] #N*m
        #输出自由度
        self.m = len(self.w[0, :])
        #权重个数
        self.w_num = len(self.w[:, 0])

        #加载dmps参数
        dmps_param = FileOpen.read(self.dmps_path)
        #时间尺度
        self.tau = dmps_param[0, 0]
        # 正则系统常数
        self.alpha = dmps_param[1, 0]
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
        self.num = len(ss)
        self.ss = ss

        #求取强迫项,用rbf求取强迫性
        f = np.zeros([self.num, self.m])
        # 求取径向基矩阵
        psi = np.zeros([self.m, self.num, self.w_num])
        for i in range(self.m):
            for j in range(self.num):
                for k in range(self.w_num):
                    psi[i, j, k] = gauss_radial_basis_function(self.h, self.c[k], self.ss[j])

        # 求取中间矩阵
        TT = np.zeros([self.m, self.num, self.w_num])
        for i in range(self.m):
            for j in range(self.num):
                psi_sum = np.sum(psi[i, j, :])
                for k in range(self.w_num):
                    TT[i, j, k] = psi[i, j, k] * self.ss[j] / psi_sum

        for i in range(self.m):
            f[:, i] = np.dot(TT, self.w[:, i])

        #求取末端位置
        XX  = np.zeros([self.num, self.m])

        x_dot = np.zeros(self.m)
        xx = np.copy(xx0)

       #采用迭代发求微分方程
        for i in range(self.num):
            for j in range(self.m):
                [xx[j], x_dot[j]] = dmps_solve_2(
                    self.tau, self.k[j], self.d[j],
                    gg[j], xx0[j], ss[i], T, f[i, j],
                    xx[j], x_dot[j])
                XX[i, j] = xx[j]
        self.xx = np.copy(XX)

    def get_plan_tcp(self):
        return self.xx

    def get_plan_joint(self, DH_0):
        #建立运动学
        kin1 = kin.GeneralKinematic(DH_0)

#dmps参数调节函数
def write_dmps_pramter():
    # 时间尺度
    tau = 10  # 所有变量时间尺度同时，保证运动统一性
    # 正则化常数
    alpha = 1
    #刚度项
    k = np.array([1000, 1000, 1000, 1000, 1000, 1000])  # 刚度项
    #阻尼项
    d = np.array([100, 100, 100, 100, 100, 500])  # 阻尼项

    # 存储dmps的模型参数
    dmps_param = np.zeros([6, 3])
    dmps_param[0, 0] = tau  # 存储在第一位，其他位用0补齐
    dmps_param[1, 0] = alpha
    dmps_param[:, 1] = k
    dmps_param[:, 2] = d

    #存储参数
    current_path = os.getcwd()
    file_path = os.path.join(current_path, "../..", "data/teaching")
    file_path = os.path.abspath(file_path)

    dmps_file_name = "dmps_parameter.txt"
    dmps_path = os.path.join(file_path, dmps_file_name)

    #写入参数
    FileOpen.write(dmps_param, dmps_path)

#rbf拟合测试
def rbf_test():
    # 获取目标路径
    current_path = os.getcwd()
    file_path = os.path.join(current_path, "../..", "data/teaching")
    file_path = os.path.abspath(file_path)

    # *****示教阶段******#
    # 读取数据
    qq_file_name = "teaching_data.txt"
    qq_path = os.path.join(file_path, qq_file_name)
    qq_demo = FileOpen.read(qq_path)[:1500, :]
    print "shape:", qq_demo.shape
    # 创建运动学
    DH_0 = np.copy(rp.DHfa_armc)
    kin1 = kin.GeneralKinematic(DH_0)

    # 获取起始点
    X0 = kin1.fkine_euler(qq_demo[0, :])
    # 获取目标点
    X_goal = kin1.fkine_euler(qq_demo[-1, :])
    # 时间系列
    T = 0.01
    num = len(qq_demo)
    tt = np.linspace(0, T * (num - 1), num)
    # 绘制数据图
    MyPlot.plot2_nd(tt, qq_demo, title="qq_demo")

    # 求取正运动学
    X_demo = np.zeros([num, 6])
    for i in range(num):
        X_demo[i, :] = kin1.fkine_euler(qq_demo[i, :])

    # 绘制数据图
    MyPlot.plot2_nd(tt, X_demo, title="X_demo")

    # *****学习阶段******#
    dmps_file_name = "dmps_parameter.txt"
    dmps_path = os.path.join(file_path, dmps_file_name)

    rbf_file_name = "rbf_parameter.txt"
    rbf_path = os.path.join(file_path, rbf_file_name)

    # 创建一个示教学习器
    m = 6
    teach_learn1 = TeachingLearn(m, dmps_path, rbf_path)

    # 获取机器人参数
    teach_learn1.get_robot_paramter(DH_0)

    # 获取rbf参数
    h = 1000  # 隐藏层个数
    teach_learn1.get_rbf_paramter(h)

    # 获取示教数据
    teach_learn1.get_teaching_data(qq_demo, X_goal, X0, T)

    # 采用最小二乘学习获取权重
    teach_learn1.learn()

    #绘制拟合图
    X_xva = teach_learn1.X_xva
    MyPlot.plot2_nd(tt, X_xva[:, :, 0], title="X")
    MyPlot.plot2_nd(tt, X_xva[:, :, 1], title="V")
    MyPlot.plot2_nd(tt, X_xva[:, :, 2], title="A")

    # 将权重写入文件
    teach_learn1.write_data()

    # 获取强迫项
    f_demo = teach_learn1.f_demo
    MyPlot.plot2_nd(tt, f_demo, title="f_demo")

    # *****示教再现阶段******#
    # 穿件示教再现器
    dmps_file_name = "dmps_parameter.txt"
    dmps_path = os.path.join(file_path, dmps_file_name)

    rbf_file_name = "rbf_parameter.txt"
    rbf_path = os.path.join(file_path, rbf_file_name)
    teach_repro1 = TeachingReproduction(dmps_path, rbf_path)

    # 规划新轨迹
    # 时间系列
    T = 0.01
    num = len(qq_demo)
    tt = np.linspace(0, T * (num - 1), num)
    print "X0:", X0
    print "X_goal:", X_goal
    xx0 = X0 + np.array([0.0, 0, -0.0, 0, 0, 0])  # 轨迹起点
    gg = X_goal + np.array([0.00, 0.00, -0.00, 0, 0, 0])  # 规划目标点
    teach_repro1.reproduction(xx0, gg, tt, T)
    X_data = teach_repro1.get_plan_tcp()

    # 获取强迫项
    f = teach_repro1.f
    # 绘制力跟踪图
    MyPlot.plot2_nd(tt, f, title="f")

    MyPlot.plot2_nd(tt, X_data, title="X_data")

#rbf拟合测试
def rbf_test1():
    # 获取目标路径
    current_path = os.getcwd()
    file_path = os.path.join(current_path, "../..", "data/teaching")
    file_path = os.path.abspath(file_path)

    # *****示教阶段******#
    # 读取数据
    T = 0.01
    num = 2000
    tt = np.linspace(0, T*(num - 1), num)
    a = 1
    Xx_demo = np.zeros([num, 2])
    Xv_demo = np.zeros([num, 2])
    Xa_demo = np.zeros([num, 2])
    Xx_demo[:, 0] = a*np.sin(np.pi/20*tt)
    Xv_demo[:, 0] = a * np.cos(np.pi / 20 * tt)*np.pi / 20
    Xa_demo[:, 0] = -a * np.sin(np.pi / 20 * tt) * (np.pi / 20)**2
    Xx_demo[:, 1] = a * np.sin(np.pi / 20 * tt) + a*0.1*np.cos(np.pi/4*tt)
    Xv_demo[:, 1] = a * np.cos(np.pi / 20 * tt) * np.pi / 20 - a*0.1*np.sin(np.pi/4*tt)*(np.pi/4)
    Xa_demo[:, 1] = -a * np.sin(np.pi / 20 * tt) * (np.pi / 20) ** 2 + a*0.1*np.cos(np.pi/4*tt)*(np.pi/4)**2
    # 绘制数据图
    MyPlot.plot2_nd(tt, Xx_demo, title="Xx_demo")
    MyPlot.plot2_nd(tt, Xv_demo, title="Xv_demo")
    MyPlot.plot2_nd(tt, Xa_demo, title="Xa_demo")

    # *****学习阶段******#
    dmps_file_name = "dmps_parameter.txt"
    dmps_path = os.path.join(file_path, dmps_file_name)

    rbf_file_name = "rbf_parameter.txt"
    rbf_path = os.path.join(file_path, rbf_file_name)

    # 创建一个示教学习器
    m = 2
    teach_learn1 = TeachingLearn1(m, dmps_path, rbf_path)

    # 获取rbf参数
    h = 1000  # 隐藏层个数
    teach_learn1.get_rbf_paramter(h)

    # 获取示教数据
    teach_learn1.get_teaching_data(Xx_demo, Xv_demo, Xa_demo,
                                   Xx_demo[-1, :], Xx_demo[0, :], T)

    # 采用最小二乘学习获取权重
    teach_learn1.learn()

    # 将权重写入文件
    teach_learn1.write_data()

    # 获取强迫项
    f_demo = teach_learn1.f_demo
    MyPlot.plot2_nd(tt, f_demo, title="f_demo")

    # *****示教再现阶段******#
    # 穿件示教再现器
    dmps_file_name = "dmps_parameter.txt"
    dmps_path = os.path.join(file_path, dmps_file_name)

    rbf_file_name = "rbf_parameter.txt"
    rbf_path = os.path.join(file_path, rbf_file_name)
    teach_repro1 = TeachingReproduction(dmps_path, rbf_path)

    # 规划新轨迹
    # 时间系列
    T = 0.01
    num = 2000
    tt = np.linspace(0, T * (num - 1), num)
    xx0 = Xx_demo[0, :] + np.array([0.0, 0.0])  # 轨迹起点
    gg =Xx_demo[-1, :] + np.array([0.10, 0.20])  # 规划目标点
    teach_repro1.reproduction(xx0, gg, tt, T)
    X_data = teach_repro1.get_plan_tcp()

    # 获取强迫项
    f = teach_repro1.f
    # 绘制力跟踪图
    MyPlot.plot2_nd(tt, f, title="f")

    MyPlot.plot2_nd(tt, X_data, title="X_data")

#局部加权拟合测试
def local_test():
    # 获取目标路径
    current_path = os.getcwd()
    file_path = os.path.join(current_path, "../..", "data/teaching")
    file_path = os.path.abspath(file_path)

    # *****示教阶段******#
    # 读取数据
    qq_file_name = "teaching_data.txt"
    qq_path = os.path.join(file_path, qq_file_name)
    qq_demo = FileOpen.read(qq_path)[0:1000, :]
    print "shape:", qq_demo.shape
    # 创建运动学
    DH_0 = np.copy(rp.DHfa_armc)
    kin1 = kin.GeneralKinematic(DH_0)

    # 获取起始点
    X0 = kin1.fkine_euler(qq_demo[0, :])
    # 获取目标点
    X_goal = kin1.fkine_euler(qq_demo[-1, :])
    # 时间系列
    T = 0.01
    num = len(qq_demo)
    tt = np.linspace(0, T * (num - 1), num)
    # 绘制数据图
    MyPlot.plot2_nd(tt, qq_demo, title="qq_demo")

    # 求取正运动学
    X_demo = np.zeros([num, 6])
    for i in range(num):
        X_demo[i, :] = kin1.fkine_euler(qq_demo[i, :])

    # 绘制数据图
    MyPlot.plot2_nd(tt, X_demo, title="X_demo")

    # *****学习阶段******#
    dmps_file_name = "dmps_parameter.txt"
    dmps_path = os.path.join(file_path, dmps_file_name)

    local_file_name = "local_parameter.txt"
    local_path = os.path.join(file_path, local_file_name)

    # 创建一个示教学习器
    m = 6
    teach_learn1 = TeachingLearnLocal(m, dmps_path, local_path)

    # 获取机器人参数
    teach_learn1.get_robot_paramter(DH_0)

    # 获取局部拟合参数
    N = 1000  # 隐藏层个数
    h = 0.1  #带宽
    teach_learn1.get_weight_num(N, h)

    # 获取示教数据
    teach_learn1.get_teaching_data(qq_demo, X_goal, X0, T)

    # 采用最小二乘学习获取权重
    teach_learn1.learn()

    # 将权重写入文件
    teach_learn1.write_data()

    # 获取强迫项
    f_demo = teach_learn1.f_demo
    MyPlot.plot2_nd(tt, f_demo, title="f_demo")

    # *****示教再现阶段******#
    # 穿件示教再现器
    dmps_file_name = "dmps_parameter.txt"
    dmps_path = os.path.join(file_path, dmps_file_name)

    rbf_file_name = "rbf_parameter.txt"
    rbf_path = os.path.join(file_path, rbf_file_name)
    teach_repro1 = TeachingReproduction(dmps_path, rbf_path)

    # 规划新轨迹
    # 时间系列
    T = 0.01
    num = len(qq_demo)
    tt = np.linspace(0, T * (num - 1), num)
    print "X0:", X0
    print "X_goal:", X_goal
    xx0 = X0 + np.array([0.0, 0, -0.0, 0, 0, 0])  # 轨迹起点
    gg = X_goal + np.array([0.09, 0.02, -0.09, 0, 0, 0])  # 规划目标点
    teach_repro1.reproduction(xx0, gg, tt, T)
    X_data = teach_repro1.get_plan_tcp()

    # 获取强迫项
    f = teach_repro1.f
    # 绘制力跟踪图
    MyPlot.plot2_nd(tt, f, title="f")

    MyPlot.plot2_nd(tt, X_data, title="X_data")

def main():
    # write_dmps_pramter()
    #local_test()
    #rbf_test1()
    rbf_test()

if __name__ == "__main__":
    main()












