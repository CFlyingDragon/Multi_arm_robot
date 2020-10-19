#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于阻抗基本函数
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.11.21
import numpy as np
from math import pi
import math
import numpy.linalg as nla
import sys
import time

import matplotlib.pyplot as plt

#自定义文件
import BaseFunction as bf
import Kinematics as kin
import RobotParameter as rp
import DataProcessing as dp
import Filter

#================创建一个适用于积分自适应的类=================#
#基于迭代求解自适应导纳方程
class IIMPController_iter(object):
    #**定义属性**#
    #阻抗参数
    M = np.zeros(6)
    B = np.zeros(6)
    K = np.zeros(6)
    I = np.zeros(6)

    #构造函数
    def __init__(self):
        # 位置、力误差
        self.Ex = np.zeros(6)   #Ex=Xr - Xd=>Xr = Xd + Ex
        self.Ef = np.zeros(6)   #Ef=Fs - Fd
        self.Ex_d = np.zeros(6)
        self.Ef_i = np.zeros(6)
        # 周期
        self.T = 0.01
        #第一次标签
        self.first_flag = True

    #**定义方法**#
    #获取控制周期
    def get_period(self, T):
        self.T = np.copy(T)

    #获取阻抗参数
    def get_imp_parameter(self,Md,Bd,Kd,Ki):
        self.M = np.copy(Md)
        self.B = np.copy(Bd)
        self.K = np.copy(Kd)
        self.I = np.copy(Ki)

    def get_robot_parameter(self, DH_0, q_max, q_min,):
        #DH参数
        self.DH0 = np.copy(DH_0)
        #关节极限
        self.qq_max = np.copy(q_max)
        self.qq_min = np.copy(q_min)
        #求取关节个数
        self.n = len(self.qq_max)

        #创建运动学类
        self.kin = kin.GeneralKinematic(DH_0, q_min, q_max)

    def get_current_joint(self, qq):
        self.qq_state = np.copy(qq)

    def get_current_force(self, F_t):
        #更新力误差
        self.Ef = F_t - self.ff_d
        print "力误差：EF", np.round(self.Ef,2)

    def get_expect_joint(self, qd):
        Xd = self.kin.fkine_euler(qd)
        self.xx_d = np.copy(Xd)

    def get_expect_pos(self, Xd):
        self.xx_d = np.copy(Xd)

    def get_expect_force(self, Fd):
        self.ff_d =np.copy(Fd)

    def int_adp_iter_solve(self, m, b, k, ki, T, ef, ef_i, ex, ex_d):
        # 求当前时刻积分项

        efk_i = ef_i + T * ef
        # 计算当前加速度
        ex_dd = (ef + ki * efk_i - b * ex_d - k * ex) / m
        print "ex_dd:", ex_dd

        # 求当前时刻速度
        exk_d = ex_d + ex_dd * T
        print "exk_d:", exk_d
        exk = ex + exk_d * T - ex_dd*T*T/2.0

        # print "积分项：",efk_i
        return [exk, exk_d, efk_i]

    def compute_imp_joint(self):
        #计算末端位置偏差
        for i in range(6):
            if(self.M[i] > math.pow(10, -6)):
                [self.Ex[i], self.Ex_d[i], self.Ef_i[i]] = self.int_adp_iter_solve(
                    self.M[i], self.B[i], self.K[i], self.I[i],
                    self.T, self.Ef[i], self.Ef_i[i],
                    self.Ex[i], self.Ex_d[i])

        #计算参考位置
        print "误差修正项：", np.round(self.Ex, 3)
        Te = self.kin.fkine(self.qq_state)
        Re = Te[0:3, 0:3]
        self.Ex[0:3] = np.dot(Re, self.Ex[0:3])
        self.Ex[3:6] = np.dot(Re, self.Ex[3:6])
        beta = 1
        Xr = self.xx_d + beta*self.Ex
        Tr = np.eye(4)
        Rr = bf.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]
        qr = self.kin.iterate_ikine_limit(self.qq_state, Tr)
        return qr

#基于迭代求解自适应导纳方程:TCP坐标系中求解
class IIMPController_iter_TCP(object):
    # **定义属性**#
    # 阻抗参数
    M = np.zeros(6)
    B = np.zeros(6)
    K = np.zeros(6)
    I = np.zeros(6)

    # 构造函数
    def __init__(self):
        # 位置、力误差
        self.Ex = np.zeros(6)  # Ex=Xr - Xd=>Xr = Xd + Ex
        self.Ef = np.zeros(6)  # Ef=Fs - Fd
        self.Ex_d = np.zeros(6)
        self.Ef_i = np.zeros(6)
        # 周期
        self.T = 0.01
        # 第一次标签
        self.first_flag = True

    # **定义方法**#
    # 获取控制周期
    def get_period(self, T):
        self.T = np.copy(T)

    # 获取阻抗参数
    def get_imp_parameter(self, Md, Bd, Kd, Ki):
        self.M = np.copy(Md)
        self.B = np.copy(Bd)
        self.K = np.copy(Kd)
        self.I = np.copy(Ki)

    def get_robot_parameter(self, DH_0, q_max, q_min):
        # DH参数
        self.DH0 = np.copy(DH_0)
        # 关节极限
        self.qq_max = np.copy(q_max)
        self.qq_min = np.copy(q_min)
        # 求取关节个数
        self.n = len(self.qq_max)

        # 创建运动学类
        self.kin = kin.GeneralKinematic(DH_0, self.qq_min, self.qq_max)

    def get_current_joint(self, qq):
        self.qq_state = np.copy(qq)

    def get_current_force(self, F_t):
        # 转换到基坐标系
        self.tcp_f = np.copy(F_t)

        # 更新力误差
        self.Ef = self.tcp_f - self.ff_d
        #print "力误差：EF", np.round(self.Ef, 2)

    def get_expect_joint(self, qd):
        Xd = self.kin.fkine_euler(qd)
        self.xx_d = np.copy(Xd)

    def get_expect_pos(self, Xd):
        self.xx_d = np.copy(Xd)

    def get_expect_force(self, Fd):
        self.ff_d = np.copy(Fd)

    def force_end_to_base(self, F):
        base_F = np.zeros(6)

        Te = self.kin.fkine(self.qq_state)
        Re = Te[0:3, 0:3]
        base_F[0:3] = np.dot(Re, F[0:3])
        base_F[3:6] = np.dot(Re, F[3:6])
        return base_F

    def int_adp_iter_solve(self, m, b, k, ki, T, ef, ef_i, ex, ex_d):
        # 求当前时刻积分项

        efk_i = ef_i + T * ef
        # 计算当前加速度
        ex_dd = (ef + ki * efk_i - b * ex_d - k * ex) / m
        print "ex_dd:", ex_dd

        # 求当前时刻速度
        exk_d = ex_d + ex_dd * T
        print "exk_d:", exk_d
        exk = ex + exk_d * T# - ex_dd * T * T / 2.0

        # print "积分项：",efk_i
        return [exk, exk_d, efk_i]

    def compute_imp_joint(self):
        # 计算末端位置偏差
        for i in range(6):
            if (self.M[i] > math.pow(10, -6)):
                [self.Ex[i], self.Ex_d[i], self.Ef_i[i]] = self.int_adp_iter_solve(
                    self.M[i], self.B[i], self.K[i], self.I[i],
                    self.T, self.Ef[i], self.Ef_i[i],
                    self.Ex[i], self.Ex_d[i])


        print "误差修正项：", np.round(self.Ex, 3)

        #转换修正项,将TCP修正量转化到基座标系
        Ex = np.zeros(6)
        Te = self.kin.fkine(self.qq_state)
        Ex[0:3] = np.dot(Te[0:3, 0:3], self.Ex[0:3])
        Ex[3:6] = np.dot(Te[0:3, 0:3], self.Ex[3:6])

        #计算参考位置
        beta = 1
        Xr = np.copy(self.xx_d + beta * Ex)
        Tr = np.eye(4)
        Rr = bf.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]

        # [qr, succeed_label] = kin.arm_angle_ikine_limit(Tr, self.qq_state, self.DH0, self.qq_min, self.qq_max)
        qr = self.kin.iterate_ikine_limit(self.qq_state, Tr)
        return qr

# 基于迭代求解自适应导纳方程:加入速度反馈
class IIMPController_iter_vel(object):
    # **定义属性**#
    # 阻抗参数
    M = np.zeros(6)
    B = np.zeros(6)
    K = np.zeros(6)
    I = np.zeros(6)

    # 构造函数
    def __init__(self):
        # 位置、力误差
        self.Ex = np.zeros(6)  # Ex=Xr - Xd=>Xr = Xd + Ex
        self.Ef = np.zeros(6)  # Ef=Fs - Fd
        self.Ex_d = np.zeros(6)
        self.Ef_i = np.zeros(6)
        # 周期
        self.T = 0.01
        # 第一次标签
        self.first_flag = True

        # 建立高增益观测器,观测速度
        self.hgo = dp.HighGainObserver()

    # **定义方法**#
    # 获取控制周期
    def get_period(self, T):
        self.T = np.copy(T)

    # 获取阻抗参数
    def get_imp_parameter(self, Md, Bd, Kd, Ki):
        self.M = np.copy(Md)
        self.B = np.copy(Bd)
        self.K = np.copy(Kd)
        self.I = np.copy(Ki)

    def get_robot_parameter(self, DH_0, q_max, q_min, ):
        # DH参数
        self.DH0 = np.copy(DH_0)
        # 关节极限
        self.qq_max = np.copy(q_max)
        self.qq_min = np.copy(q_min)
        # 求取关节个数
        self.n = len(self.qq_max)

        # 创建运动学类
        self.kin = kin.GeneralKinematic(DH_0)

    def get_current_joint(self, qq):
        self.qq_state = np.copy(qq)
        X = self.kin.fkine_euler(qq)
        if(self.first_flag):
            self.Xx_list = np.zeros([3, 6])
            self.Xx_list[0, :] = X
            self.Xx_list[1, :] = X
            self.Xx_list[2, :] = X
            self.Xv_list = np.zeros([2, 6])
        else:
            self.Xx_list[0, :] = self.Xx_list[1, :]
            self.Xx_list[1, :] = self.Xx_list[2, :]
            self.Xx_list[2, :] = X

        Xv = np.zeros(6)
        for i in range(6):
            Xv[i] = self.hgo.observer(self.Xx_list[:, i], self.Xv_list[:, i])
        self.Xv_list[0, :] = self.Xv_list[1, :]
        self.Xv_list[1, :] = Xv

    def get_current_force(self, F_t):
        # 转换到基坐标系
        self.base_f = self.force_end_to_base(F_t)

        # 更新力误差
        self.Ef = self.base_f - self.ff_d
        print "力误差：EF", np.round(self.Ef, 2)

    def get_expect_joint(self, qd):
        Xd = self.kin.fkine_euler(qd)
        self.xx_d = np.copy(Xd)

    def get_expect_pos(self, Xd):
        self.xx_d = np.copy(Xd)

    def get_expect_force(self, Fd):
        self.ff_d = self.force_end_to_base(Fd)

    def force_end_to_base(self, F):
        base_F = np.zeros(6)

        Te = self.kin.fkine(self.qq_state)
        Re = Te[0:3, 0:3]
        base_F[0:3] = np.dot(Re, F[0:3])
        base_F[3:6] = np.dot(Re, F[3:6])
        return base_F

    def int_adp_iter_solve(self, m, b, k, ki, T, ef, ef_i, ex, ex_d):
        # 求当前时刻积分项

        efk_i = ef_i + T * ef
        # 计算当前加速度
        ex_dd = (ef + ki * efk_i - b * ex_d - k * ex) / m
        # print "ex_dd:", ex_dd

        # 求当前时刻速度
        exk_d = ex_d + ex_dd * T
        # print "exk_d:", exk_d
        exk = ex + exk_d * T - ex_dd * T * T / 2.0

        # print "积分项：",efk_i
        return [exk, exk_d, efk_i]

    def compute_imp_joint(self):
        # 计算末端位置偏差
        for i in range(6):
            if (self.M[i] > math.pow(10, -6)):
                [self.Ex[i], self.Ex_d[i], self.Ef_i[i]] = self.int_adp_iter_solve(
                    self.M[i], self.B[i], self.K[i], self.I[i],
                    self.T, self.Ef[i], self.Ef_i[i],
                    self.Ex[i], -self.Xv_list[-1, i])

        # 计算参考位置
        print "误差修正项：", np.round(self.Ex, 3)
        beta = 0.5
        Xr = np.copy(self.xx_d + beta * self.Ex)
        Tr = np.eye(4)
        Rr = bf.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]

        # [qr, succeed_label] = kin.arm_angle_ikine_limit(Tr, self.qq_state, self.DH0, self.qq_min, self.qq_max)
        qr = self.kin.iterate_ikine(self.qq_state, Tr)

        return qr

#基于差分方程求解自适应导纳方程:调试成功,理论上效果更佳
class IIMPController_diff(object):
    #**定义属性**#
    #阻抗参数
    M = np.zeros(6)
    B = np.zeros(6)
    K = np.zeros(6)
    I = np.zeros(6)

    def __init__(self):
        #位置、力误差
        self.E_f_list = np.zeros([4, 6])
        self.E_x_list = np.zeros([3, 6])
        self.qq_state = np.zeros(7)

        #周期
        self.T = 0.01
        self.init_flag = True
        self.expect_joint_flag = False

    #**定义方法**#
    #获取控制周期
    def get_period(self, T):
        self.T = np.copy(T)

    #获取阻抗参数
    def get_imp_parameter(self, Md, Bd, Kd, Ki):
        self.M = np.copy(Md)
        self.B = np.copy(Bd)
        self.K = np.copy(Kd)
        self.I = np.copy(Ki)
        self.diff_paramter(Md, Bd, Kd, Ki)

    def get_robot_parameter(self, DH_0, q_max, q_min):
        #DH参数
        self.DH0 = np.copy(DH_0)
        #关节极限
        self.qq_max = np.copy(q_max)
        self.qq_min = np.copy(q_min)
        #求取关节个数
        self.n = len(self.qq_max)

        #创建运动学类
        self.kin = kin.GeneralKinematic(DH_0, q_min, q_max)

    def get_current_joint(self, qq):
        self.qq_state = np.copy(qq)

    def get_current_force(self, F_t):
        Ef = F_t - self.ff_d
        self.E_f_list[3, :] = self.E_f_list[2, :]
        self.E_f_list[2, :] = self.E_f_list[1, :]
        self.E_f_list[1, :] = self.E_f_list[0, :]
        self.E_f_list[0, :] = Ef

    def get_expect_joint(self, qd):
        self.expect_joint_flag = True
        Td = self.kin.fkine(qd)
        self.Td = np.copy(Td)

    def get_expect_pos(self, Xd):
        self.expect_joint_flag = False
        self.xx_d = np.copy(Xd)

    def get_expect_force(self, Fd):
        self.ff_d = np.copy(Fd)

    #计算参数差分方程参数
    def diff_paramter(self, M, B, K, Ki):
        #计算参数矩阵D=[M, B*T, K*T**2, Ki*T]
        D = np.ones([6, 5])
        D[:, 0] = M
        D[:, 1] = B*self.T
        D[:, 2] = K*self.T*self.T
        D[:, 3] = Ki*self.T

        #z函数分子：C=[B0, B1, B2, B3]
        C_c = np.array([[1, 2],
                     [3, 2],
                     [3, -2],
                     [1, -2.0]]).T
        self.C = np.dot(D[:, 3:5], C_c)
        #print "C：", self.C

        #z函数分母:A = [A0, A1, A2, A3]
        A_c = np.array([[4, 2, 1],
                        [-12, -2, 1],
                        [12, -2, -1],
                        [-4, 2, -1.0]]).T
        self.A = np.dot(D[:, 0:3], A_c)
        #print "A:", self.A

    #基于双曲变换求取积分自适应导纳方程,单自由度方程
    def int_adp_imc_diff_eq(self, Ef, Ex):
        '''
        :param ef: 4个时刻的力偏差
        :param ex: 3个时刻的位置误差
        :return:
        '''
        E = np.zeros(6)
        for i in range(6):
            if (abs(self.A[i, 0]) < math.pow(10,-6)):
                E[i] = 0.0
            else:
                E[i] = (0.5*self.T*self.T*np.dot(self.C[i, :], Ef[:, i])\
                        - np.dot(self.A[i, 1:], Ex[:, i]))/self.A[i, 0]
        return E

    def compute_imp_joint(self):
        # print "位置误差：", np.round(self.E_x_list, 3)
        # print "力误差：", np.round(self.E_f_list, 3)
        #计算末端位置偏差
        E = self.int_adp_imc_diff_eq(self.E_f_list, self.E_x_list)
        self.E_x_list[2, :] = self.E_x_list[1, :]
        self.E_x_list[1, :] = self.E_x_list[0, :]
        self.E_x_list[0, :] = E

        #计算参考位置:转换到基座标
        print "误差修正项：", np.round(E, 6)
        Te = self.kin.fkine(self.qq_state)
        Re = Te[0:3, 0:3]
        base_E = np.zeros(6)
        base_E[0:3] = np.dot(Re, E[0:3])
        base_E[3:6] = np.dot(Re, E[3:6])

        Tr = np.eye(4)
        if(self.expect_joint_flag):
            Tr[0:3, 0:3] = self.Td[0:3, 0:3]
            Tr[0:3, :] = self.Td[0:3, 3] - E[0:3]
        else:
            Xr = self.xx_d - E
            Tr[0:3, 0:3] = bf.euler_zyx2rot(Xr[3:6])
            Tr[0:3, 3] = Xr[0:3]

        qr = self.kin.iterate_ikine_limit(self.qq_state, Tr)

        return qr

# 基于差分方程求解自适应导纳方程:加入加入设计好的低通滤波器
class IIMPController_diff_filter(object):
    def __init__(self, wc=0.001, N=30):
        #位置、力误差
        self.E_f_list = np.zeros([4, 6])
        self.E_x_list = np.zeros([3, 6])
        self.qq_state = np.zeros(7)

        #周期
        self.T = 0.01
        self.init_flag = True
        self.wc = wc
        self.N = N
        self.FIR_filt = Filter.FIRFilter(wc, N)

    #**定义方法**#
    #获取控制周期
    def get_period(self, T):
        self.T = np.copy(T)

    #获取阻抗参数
    def get_imp_parameter(self, Md, Bd, Kd, Ki):
        self.M = np.copy(Md)
        self.B = np.copy(Bd)
        self.K = np.copy(Kd)
        self.I = np.copy(Ki)
        self.diff_paramter(Md, Bd, Kd, Ki)

    def get_robot_parameter(self, DH_0, q_max, q_min):
        #DH参数
        self.DH0 = np.copy(DH_0)
        #关节极限
        self.qq_max = np.copy(q_max)
        self.qq_min = np.copy(q_min)
        #求取关节个数
        self.n = len(self.qq_max)

        #创建运动学类
        self.kin = kin.GeneralKinematic(DH_0, q_min, q_max)

    def get_current_joint(self, qq):
        self.qq_state = np.copy(qq)

    def get_current_force(self, F_t):
        if (self.init_flag):
            self.FIR_filt.set_input_init(F_t)
            self.FIR_filt.set_hanning_filter()
            self.init_flag = False
            # 加入滤波
        F_t = self.FIR_filt.hanning_filter(F_t)
        Ef = F_t - self.ff_d
        self.E_f_list[3, :] = self.E_f_list[2, :]
        self.E_f_list[2, :] = self.E_f_list[1, :]
        self.E_f_list[1, :] = self.E_f_list[0, :]
        self.E_f_list[0, :] = Ef

    def get_expect_joint(self, qd):
        Xd = self.kin.fkine_euler(qd)
        self.xx_d = np.copy(Xd)

    def get_expect_pos(self, Xd):
        self.xx_d = np.copy(Xd)

    def get_expect_force(self, Fd):
        self.ff_d = np.copy(Fd)

    #计算参数差分方程参数
    def diff_paramter(self, M, B, K, Ki):
        #计算参数矩阵D=[M, B*T, K*T**2, Ki*T]
        D = np.ones([6, 5])
        D[:, 0] = M
        D[:, 1] = B*self.T
        D[:, 2] = K*self.T*self.T
        D[:, 3] = Ki*self.T

        #z函数分子：C=[B0, B1, B2, B3]
        C_c = np.array([[1, 2],
                     [3, 2],
                     [3, -2],
                     [1, -2.0]]).T
        self.C = np.dot(D[:, 3:5], C_c)
        #print "C：", self.C

        #z函数分母:A = [A0, A1, A2, A3]
        A_c = np.array([[4, 2, 1],
                        [-12, -2, 1],
                        [12, -2, -1],
                        [-4, 2, -1.0]]).T
        self.A = np.dot(D[:, 0:3], A_c)
        #print "A:", self.A

    #基于双曲变换求取积分自适应导纳方程,单自由度方程
    def int_adp_imc_diff_eq(self, Ef, Ex):
        '''
        :param ef: 4个时刻的力偏差
        :param ex: 3个时刻的位置误差
        :return:
        '''
        E = np.zeros(6)
        for i in range(6):
            if (abs(self.A[i, 0]) < math.pow(10,-6)):
                E[i] = 0.0
            else:
                E[i] = (0.5*self.T*self.T*np.dot(self.C[i, :], Ef[:, i])\
                        - np.dot(self.A[i, 1:], Ex[:, i]))/self.A[i, 0]
        return E

    def compute_imp_joint(self):
        # print "位置误差：", np.round(self.E_x_list, 3)
        # print "力误差：", np.round(self.E_f_list, 3)
        #计算末端位置偏差
        E = self.int_adp_imc_diff_eq(self.E_f_list, self.E_x_list)
        self.E_x_list[2, :] = self.E_x_list[1, :]
        self.E_x_list[1, :] = self.E_x_list[0, :]
        self.E_x_list[0, :] = E

        #计算参考位置:转换到基座标
        print "误差修正项：", np.round(E, 6)
        Te = self.kin.fkine(self.qq_state)
        Re = Te[0:3, 0:3]
        base_E = np.zeros(6)
        base_E[0:3] = np.dot(Re, E[0:3])
        base_E[3:6] = np.dot(Re, E[3:6])

        beta = 1
        Xr = self.xx_d - beta*E
        qr = self.kin.iterate_ikine_limit_xyz(self.qq_state, Xr)

        return qr

#================创建一个刚度评估项类函数=================#
class StiffnessEvaluation(object):
    def __init__(self):
        self.num = 10

    #**定义方法**#
    def get_robot_parameter(self, DH_0, q_max, q_min,):
        #DH参数
        self.DH0 = np.copy(DH_0)
        #关节极限
        self.qq_max = np.copy(q_max)
        self.qq_min = np.copy(q_min)
        #求取关节个数
        self.n = len(self.qq_max)

        #创建运动学类
        self.kin = kin.GeneralKinematic(DH_0)

    def get_pos1_joint(self, qq):
        #采集10个关节角求平均值
        self.pos1_qq = np.zeros(self.n)
        for i in range(self.n):
            self.pos1_qq[i] = sum(qq[:, i])/self.num

    def get_pos1_force(self, F_t):
        #转换到基坐标系
        self.pos1_f = self.force_end_to_base(F_t, self.pos1_qq)

    def get_pos2_joint(self, qq):
        # 采集10个关节角求平均值
        self.pos2_qq = np.zeros(self.n)
        for i in range(self.n):
            self.pos2_qq[i] = sum(qq[:, i]) / self.num

    def get_pos2_force(self, F_t):
        # 转换到基坐标系
        self.pos2_f = self.force_end_to_base(F_t, self.pos2_qq)

    def force_end_to_base(self, F, qq):
        base_F = np.zeros(6)

        Te = self.kin.fkine(qq)
        Re = Te[0:3, 0:3]
        base_F[0:3] = np.dot(Re, F[0:3])
        base_F[3:6] = np.dot(Re, F[3:6])
        return base_F

    #计算等效刚度
    def compute_Stiffness(self):
        #求取末端位姿
        X1 = self.kin.fkine_euler(self.pos1_qq)
        X2 = self.kin.fkine_euler(self.pos2_qq)

        #计算偏差
        X = X2 - X1
        F = self.pos2_f - self.pos1_f
        #计算等效刚度
        K = np.zeros(6)
        for i in range(6):
            if(abs(X[i]) > pow(10, -6)):
                K[i] = F[i]/X[i]
        return K

#====================变参数设计========================#
class IMPParamater(object):
    def __init__(self):
        pass

    def get_init_imp_paramter(self, M, B, K, I):
        '''
        :param M: 基准阻抗参数,初时刻阻抗参数
        :param B:
        :param K:
        :param I:
        '''
        self.M_init = np.copy(M)
        self.B_init = np.copy(B)
        self.K_init = np.copy(K)
        self.I_init = np.copy(I)

    def get_diff_imp_paramter(self,  dM, dB, dK, dI):
        '''
        :param dM: 动态响应时刻阻抗参数差值
        :param dB:
        :param dK:
        :param dI:
        '''
        self.M_diff = np.copy(dM)
        self.B_diff = np.copy(dB)
        self.K_diff = np.copy(dK)
        self.I_diff = np.copy(dI)

        #动态响应段参数
        self.M = self.M_init + self.M_diff
        self.B = self.B_init + self.B_diff
        self.K = self.K_init + self.K_diff
        self.I = self.I_init + self.I_diff

    def sfunc(self, x, k=1, a=400, b=1, m=0.005):
        y = k / (1 + math.exp(m*(a - b * x)))
        return y

    def get_function_paramter(self, a, b):
        '''
        :param a: 切变点1,以周期数为单位
        :param b: 切变点2
        '''
        self.a = a
        self.b = b

        #计算函数
        l = b - a + 1
        self.f = np.zeros(l)
        for i in range(l):
            self.f[i] = float(i)/l

    def out_sfun_paramter(self, k):
        f = self.sfunc(k, a= (self.a +self.b)/2)
        # 计算方程
        M = self.M_init + f * self.M_diff
        B = self.B_init + f * self.B_diff
        K = self.K_init + f * self.K_diff
        I = self.I_init + f * self.I_diff

        return [M, B, K, I]

    def out_expect_imp_paramter(self, k):
        '''
        :param k:
        :return:
        '''
        if(k > self.b):
            return [self.M, self.B, self.K, self.I]
        if(k < self.a):
            return [self.M_init, self.B_init, self.K_init, self.I_init]

        #计算方程
        M = self.M_init + self.f[k - self.a] * self.M_diff
        B = self.B_init + self.f[k - self.a] * self.B_diff
        K = self.K_init + self.f[k - self.a] * self.K_diff
        I = self.I_init + self.f[k - self.a] * self.I_diff

        return [M, B, K, I]

#=============测试方程=============#
def test():
    # 创建阻抗
    #imp = IIMPController_iter()
    imp = IIMPController_diff()

    #创建滤波器
    wc = 0.001
    N = 30
    my_filter = Filter.FIRFilter(wc=wc, N=N)
    my_filter.set_input_init(np.zeros(6))
    my_filter.set_hanning_filter()

    # 输入阻抗参数
    M = np.array([0, 0, 1, 0, 0, 0])
    B = np.array([0, 0, 600, 0, 0, 0])
    K = np.array([0, 0, 0, 0, 0, 0])
    I = np.array([0, 0, 20, 0, 0, 0])

    # 控制周期
    T = 0.01
    imp.get_period(T)

    # 输入DH参数
    DH = rp.DHfa_armc
    imp.get_robot_parameter(DH, rp.q_max_armc, rp.q_min_armc)

    num_imp = 2001
    num_init = 1000
    num = num_init + num_imp
    k = 10

    # 环境刚度参数
    ke = 30000

    '''
    多次实验表明，需要解决两个问题
    (1)从初始位置到接触的瞬间冲击问题：超调量问题
    (2)接触后,环境曲面变化的适应问题：快速响应性
    '''
    # 建立期望轨迹
    ld = 0.05
    Xd = np.array([0.42, 0, 0.000, 0, np.pi, 0])
    t = np.linspace(0, T * (num - 1), num)
    tt = np.linspace(0, T * (num_imp - 1), num_imp)
    Xd_array = np.zeros([num, 6])
    for i in range(num):
        if (i < num_init):
            Xd_array[i, :] = Xd
        else:
            Xd_array[i, :] = Xd
            Xd_array[i, 0] = Xd[0] + ld * np.sin(0.5 * np.pi / k * tt[i - num_init])

    # 建立实际环境
    le = 0.01
    led = - 0
    Xe = np.copy(Xd)
    Xe_array = np.zeros([num, 6])
    for i in range(num):
        if (i < num_init):
            Xe_array[i, :] = Xe
        else:
            Xe_array[i, :] = Xe
            Xe_array[i, 0] = Xe[0] + ld * np.sin(0.5 * np.pi / k * tt[i - num_init])
            Xe_array[i, 2] = Xe[2] - le * np.sin(2 * np.pi / k * tt[i - num_init]) - led

    # 建立期望力
    lf = 10
    Fd = np.array([0, 0, -10.0, 0, 0.0, 0])
    Fd_array = np.zeros([num, 6])
    t_init = (num_init-1)*T
    [Fd_array[0:num_init, 2], _, _] = bf.interp5rdPoly1(0, 0, 0, Fd[2], 0, 0, t_init, T)
    Fd_array[num_init:, :] = np.dot(np.ones([num_imp, 6]), np.diag(Fd))

    plt.figure(1)
    plt.plot(t, Fd_array[:, 2], label='Fz', color='b')
    plt.title("Fd")
    plt.xlabel("t/s")
    plt.ylabel("Fz/N")
    plt.legend()

    plt.figure(2)
    plt.plot(t, 1000*Xe_array[:, 2], label='Xz', color='b')
    plt.title("Xe")
    plt.xlabel("t/s")
    plt.ylabel("Xz/mm")
    plt.legend()
    #plt.show()

    # 建立运动学
    kin1 = kin.GeneralKinematic(DH)

    # 获取初始状态
    # 初始位置
    q_guess = np.array([0.0, 30, 0, 90, 0, 60, 0])
    Te = np.eye(4)
    Te[0:3, 0:3] = bf.euler_zyx2rot(Xd_array[0, 3:6])
    Te[0:3, 3] = Xd_array[0, 0:3]
    qq_state = kin1.iterate_ikine(q_guess, Te)

    print "qq_state:", np.around(qq_state, 3)
    # 初始力
    Fs = np.zeros(6)
    Ff = np.zeros(6)

    Fs_array = np.zeros([num, 6])
    Ff_array = np.zeros([num, 6])

    # 机械臂末端位置
    Xs_array = np.zeros([num, 6])
    # 输入阻抗参数
    imp.get_imp_parameter(M, B, K, I)

    # 合成阻抗轨迹
    for i in range(num):
        # 读取期望位姿和关节角
        imp.get_expect_pos(Xd_array[i, :])
        imp.get_current_joint(qq_state)
        # 读取当前关节角和力
        imp.get_expect_force(Fd_array[i, :])
        imp.get_current_force(Fs)
        # 计算修正关节角
        qr = imp.compute_imp_joint()

        # 建设机器人底层能跟踪
        qq_state = np.copy(qr)
        Xs = kin1.fkine_euler(qq_state)

        Xs_array[i, :] = Xs

        # 反馈力
        if (Xe_array[i, 2] > Xs[2]):
            Fs = -ke * (Xe_array[i, :] - Xs)# + np.random.randn(6)
            Ff = my_filter.hanning_filter(Fs)
        else:
            Fs = np.zeros(6)
            Ff = my_filter.hanning_filter(Fs)
        Fs_array[i, :] = np.copy(Fs)
        Ff_array[i, :] = np.copy(Ff)

    # 绘制力跟踪图
    plt.figure(3)
    plt.plot(t, Fs_array[:, 2], label='Fz', color='r')
    plt.title("Fs = Fs_d+random(1)")
    plt.xlabel("t/s")
    plt.ylabel("F/N")
    plt.legend()

    plt.figure(4)
    plt.plot(t, Ff_array[:, 2], label='Fz', color='r')
    plt.title("Ff")
    plt.xlabel("t/s")
    plt.ylabel("F/N")
    plt.legend()

    plt.figure(5)
    plt.plot(t, Fd_array[:, 2] - Fs_array[:, 2], label='Fz_err', color='b',
             linestyle=':', marker='o', markerfacecolor='r', markersize=2)
    plt.title("Fz_error")
    plt.xlabel("t/s")
    plt.ylabel("F/N")
    plt.legend()

    plt.figure(6)
    plt.plot(t, Fd_array[:, 2] - Ff_array[:, 2], label='Fz_err', color='b',
             linestyle=':', marker='o', markerfacecolor='r', markersize=2)
    plt.title("Fz_error")
    plt.xlabel("t/s")
    plt.ylabel("F/N")
    plt.legend()

    plt.figure(7)
    plt.plot(t, Xe_array[:, 2], label='Xz', color='b',
             linestyle=':', marker='o', markerfacecolor='r', markersize=2)
    plt.title("Xz")
    plt.xlabel("t/s")
    plt.ylabel("x/m")
    plt.legend()
    plt.show()

#=============测试方程=============#
def test_armt():
    # 创建阻抗
    #imp = IIMPController_iter()
    imp = IIMPController_diff_filter()

    #创建滤波器
    wc = 0.001
    N = 30
    my_filter = Filter.FIRFilter(wc=wc, N=N)
    my_filter.set_input_init(np.zeros(6))
    my_filter.set_hanning_filter()

    # 输入阻抗参数
    M = np.array([0, 0, 1, 0, 0, 0])
    B = np.array([0, 0, 2000, 0, 0, 0])
    K = np.array([0, 0, 0, 0, 0, 0])
    I = np.array([0, 0, 10, 0, 0, 0])

    # 控制周期
    T = 0.01
    imp.get_period(T)

    # 输入DH参数
    DH = rp.DHf_armt
    q_max= rp.q_max_armt
    q_min = rp.q_min_armt
    imp.get_robot_parameter(DH, q_max, q_min)

    num_imp = 2001
    num_init = 1000
    num = num_init + num_imp
    k = 10

    # 环境刚度参数
    ke = 80000

    '''
    多次实验表明，需要解决两个问题
    (1)从初始位置到接触的瞬间冲击问题：超调量问题
    (2)接触后,环境曲面变化的适应问题：快速响应性
    '''
    # 建立期望轨迹
    ld = 0.05
    Xd = np.array([0.42, 0, 0.000, 0, np.pi, 0])
    t = np.linspace(0, T * (num - 1), num)
    tt = np.linspace(0, T * (num_imp - 1), num_imp)
    Xd_array = np.zeros([num, 6])
    for i in range(num):
        if (i < num_init):
            Xd_array[i, :] = Xd
        else:
            Xd_array[i, :] = Xd
            Xd_array[i, 0] = Xd[0] + ld * np.sin(0.5 * np.pi / k * tt[i - num_init])

    # 建立实际环境
    le = 0.01
    led = - 0
    Xe = np.copy(Xd)
    Xe_array = np.zeros([num, 6])
    for i in range(num):
        if (i < num_init):
            Xe_array[i, :] = Xe
        else:
            Xe_array[i, :] = Xe
            Xe_array[i, 0] = Xe[0] + ld * np.sin(0.5 * np.pi / k * tt[i - num_init])
            Xe_array[i, 2] = Xe[2] - le * np.sin(2 * np.pi / k * tt[i - num_init]) - led

    # 建立期望力
    lf = 10
    Fd = np.array([0, 0, -10.0, 0, 0.0, 0])
    Fd_array = np.zeros([num, 6])
    t_init = (num_init-1)*T
    [Fd_array[0:num_init, 2], _, _] = bf.interp5rdPoly1(0, 0, 0, Fd[2], 0, 0, t_init, T)
    Fd_array[num_init:, :] = np.dot(np.ones([num_imp, 6]), np.diag(Fd))

    plt.figure(1)
    plt.plot(t, Fd_array[:, 2], label='Fz', color='b')
    plt.title("Fd")
    plt.xlabel("t/s")
    plt.ylabel("Fz/N")
    plt.legend()

    plt.figure(2)
    plt.plot(t, 1000*Xe_array[:, 2], label='Xz', color='b')
    plt.title("Xe")
    plt.xlabel("t/s")
    plt.ylabel("Xz/mm")
    plt.legend()
    #plt.show()

    # 建立运动学
    kin1 = kin.GeneralKinematic(DH, q_min, q_max)

    # 获取初始状态
    # 初始位置
    q_guess = np.array([0.0, 30, 0, 90, 0, 60, 0])
    Te = np.eye(4)
    Te[0:3, 0:3] = bf.euler_zyx2rot(Xd_array[0, 3:6])
    Te[0:3, 3] = Xd_array[0, 0:3]
    qq_state = kin1.iterate_ikine(q_guess, Te)

    print "qq_state:", np.around(qq_state, 3)
    # 初始力
    Fs = np.zeros(6)
    Ff = np.zeros(6)

    Fs_array = np.zeros([num, 6])
    Ff_array = np.zeros([num, 6])

    # 机械臂末端位置
    Xs_array = np.zeros([num, 6])
    # 输入阻抗参数
    imp.get_imp_parameter(M, B, K, I)

    # 合成阻抗轨迹
    for i in range(num):
        # 读取期望位姿和关节角
        imp.get_expect_pos(Xd_array[i, :])
        imp.get_current_joint(qq_state)
        # 读取当前关节角和力
        imp.get_expect_force(Fd_array[i, :])
        imp.get_current_force(Fs)
        # 计算修正关节角
        qr = imp.compute_imp_joint()

        # 建设机器人底层能跟踪
        qq_state = np.copy(qr)
        Xs = kin1.fkine_euler(qq_state)

        Xs_array[i, :] = Xs

        # 反馈力
        if (Xe_array[i, 2] > Xs[2]):
            Fs = -ke * (Xe_array[i, :] - Xs)# + np.random.randn(6)
            Ff = my_filter.hanning_filter(Fs)
        else:
            Fs = np.zeros(6)
            Ff = my_filter.hanning_filter(Fs)
        Fs_array[i, :] = np.copy(Fs)
        Ff_array[i, :] = np.copy(Ff)

    # 绘制力跟踪图
    plt.figure(3)
    plt.plot(t, Fs_array[:, 2], label='Fz', color='r')
    plt.title("Fs = Fs_d+random(1)")
    plt.xlabel("t/s")
    plt.ylabel("F/N")
    plt.legend()

    plt.figure(4)
    plt.plot(t, Ff_array[:, 2], label='Fz', color='r')
    plt.title("Ff")
    plt.xlabel("t/s")
    plt.ylabel("F/N")
    plt.legend()

    plt.figure(5)
    plt.plot(t, Fd_array[:, 2] - Fs_array[:, 2], label='Fz_err', color='b',
             linestyle=':', marker='o', markerfacecolor='r', markersize=2)
    plt.title("Fz_error")
    plt.xlabel("t/s")
    plt.ylabel("F/N")
    plt.legend()

    plt.figure(6)
    plt.plot(t, Fd_array[:, 2] - Ff_array[:, 2], label='Fz_err', color='b',
             linestyle=':', marker='o', markerfacecolor='r', markersize=2)
    plt.title("Fz_error")
    plt.xlabel("t/s")
    plt.ylabel("F/N")
    plt.legend()

    plt.figure(7)
    plt.plot(t, Xe_array[:, 2], label='Xz', color='b',
             linestyle=':', marker='o', markerfacecolor='r', markersize=2)
    plt.title("Xz")
    plt.xlabel("t/s")
    plt.ylabel("x/m")
    plt.legend()
    plt.show()

def main():
    #创建阻抗
    #imp = IIMPController_iter()
    imp = IIMPController_diff()
    #创建阻抗参数调节器
    imp_param = IMPParamater()
    #输入阻抗参数
    M = np.zeros(6)
    M[2] = 1
    dM = -0
    B = np.zeros(6)
    B[2] = 600
    dB = -200
    K = np.zeros(6)
    K[2] = 0
    dK = 0
    I = np.zeros(6)
    I[2] = 5
    dI = 25

    a = 1000
    b = 1000

    imp_param.get_init_imp_paramter(M[2], B[2], K[2], I[2])
    imp_param.get_diff_imp_paramter(dM, dB, dK, dI)
    imp_param.get_function_paramter(a, b)

    #控制周期
    T = 0.01
    imp.get_period(T)

    #输入DH参数
    imp.get_robot_parameter(rp.DHfa_armc, rp.q_max_armc, rp.q_min_armc)

    num_imp = 2001
    num_init = 1000
    num = num_init + num_imp
    k = 5

    # 环境刚度参数
    ke = 30000

    '''
    多次实验表明，需要解决两个问题
    (1)从初始位置到接触的瞬间冲击问题：超调量问题
    (2)接触后,环境曲面变化的适应问题：快速响应性
    '''

    #建立期望轨迹
    ld = 0
    Xd = np.array([0.40, 0, 0, 0, np.pi, 0])
    t = np.linspace(0, T*(num - 1), num)
    tt = np.linspace(0, T*(num_imp - 1), num_imp)
    Xd_array = np.zeros([num, 6])
    for i in range(num):
        if(i<num_init):
            Xd_array[i, :] = Xd
            Xd_array[i, 0] = Xd[0] + ld * np.sin(0.5*np.pi / k * tt[0])
        else:
            Xd_array[i, :] = Xd
            Xd_array[i, 0] = Xd[0] + ld * np.sin(0.5*np.pi / k * tt[i-num_init])

    #建立实际环境
    le = 0.000
    led = - 0
    Xe = np.copy(Xd)
    Xe_array = np.zeros([num, 6])
    for i in range(num):
        if(i<num_init):
            Xe_array[i, :] = Xe
            Xe_array[i, 0] = Xe[0] + ld * np.sin(0.5*np.pi / k * tt[0])
            Xe_array[i, 2] = Xe[2] - le * np.cos(0.1*np.pi / k * tt[0]) - led
        else:
            Xe_array[i, :] = Xe
            Xe_array[i, 0] = Xe[0] + ld * np.sin(0.5*np.pi/k * tt[i-num_init])
            Xe_array[i, 2] = Xe[2] - le * np.cos(2*np.pi/k * tt[i-num_init]) - led

    #建立期望力
    lf = -10
    Fd = np.array([0, 0, -15.0, 0, 0.0, 0])
    Fd_array = np.zeros([num, 6])
    for i in range(num):
        if(i<num_init):
            Fd_array[i, :] = Fd
            Fd_array[i, 2] = Fd[2] + lf * np.sin(2 * np.pi/k * tt[0])
        else:
            Fd_array[i, :] = Fd
            Fd_array[i, 2] = Fd[2] + lf * np.sin(2*np.pi/k * tt[i-num_init])

    #建立运动学
    kin1 = kin.GeneralKinematic(rp.DHfa_armc)

    #获取初始状态
    #初始位置
    q_guess = np.array([0.0, 30, 0, 90, 0, 60, 0])
    Te = np.eye(4)
    Te[0:3, 0:3] = bf.euler_zyx2rot(Xd_array[0, 3:6])
    Te[0:3, 3] = Xd_array[0, 0:3]
    qq_state = kin1.iterate_ikine(q_guess, Te)

    print "qq_state:", np.around(qq_state, 3)
    #初始力
    Fs = np.zeros(6)

    Fs_array = np.zeros([num, 6])

    #机械臂末端位置
    Xs_array = np.zeros([num, 6])

    #合成阻抗轨迹
    for i in range(num):
        #变阻抗参数
        #[M[2], B[2], K[2], I[2]] = imp_param.out_expect_imp_paramter(i)
        #[M[2], B[2], K[2], I[2]] = imp_param.out_sfun_paramter(i)

        #输入阻抗参数
        imp.get_imp_parameter(M, B, K, I)

        # 读取期望位姿和关节角
        imp.get_expect_pos(Xd_array[i, :])
        imp.get_current_joint(qq_state)
        # 读取当前关节角和力
        imp.get_expect_force(Fd_array[i, :])
        imp.get_current_force(Fs)
        # 计算修正关节角
        qr = imp.compute_imp_joint()

        #建设机器人底层能跟踪
        qq_state = np.copy(qr)
        Xs = kin1.fkine_euler(qq_state)

        Xs_array[i, :] = Xs

        #print "Xs_z:", Xs[2]

        #反馈力
        if (Xe_array[i, 2] >  Xs[2] ):
            Fs = -ke*(Xe_array[i, :] - Xs)
        else:
            Fs = np.zeros(6)
        #print "Fs_z:", Fs[2]
        Fs_array[i, :] = np.copy(Fs)

    #绘制力跟踪图
    plt.figure(1)
    plt.plot(t, Fs_array[:, 2], label='Fz', color='r')
    plt.title("Fz")
    plt.xlabel("t/s")
    plt.ylabel("F/N")
    plt.legend()

    plt.figure(2)
    plt.plot(t, Fd_array[:, 2] - Fs_array[:, 2], label='Fz_err', color='b',
             linestyle=':', marker='o', markerfacecolor='r', markersize=2)
    plt.title("Fz_error")
    plt.xlabel("t/s")
    plt.ylabel("F/N")
    plt.legend()


    plt.figure(3)
    plt.plot(t, Xe_array[:, 2], label='Xs', color='b',
    linestyle=':', marker='o', markerfacecolor='r', markersize=2)
    plt.title("Xs")
    plt.xlabel("t/s")
    plt.ylabel("x/m")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    #main()
    test_armt()
    print "finish!"