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
        self.kin = kin.GeneralKinematic(DH_0)

    def get_current_joint(self, qq):
        self.qq_state = np.copy(qq)

    def get_current_force(self, F_t):
        #转换到基坐标系
        self.base_f = self.force_end_to_base(F_t)

        #更新力误差
        self.Ef = self.base_f - self.ff_d
        print "力误差：EF", np.round(self.Ef,2)

    def get_expect_joint(self, qd):
        Xd = self.kin.fkine_euler( qd)
        self.xx_d = np.copy(Xd)

    def get_expect_pos(self, Xd):
        self.xx_d = np.copy(Xd)

    def get_expect_force(self, Fd):
        self.ff_d =self.force_end_to_base(Fd)

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
        beta = 1
        Xr = np.copy(self.xx_d + beta*self.Ex)
        Tr = np.eye(4)
        Rr = bf.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]

        #[qr, succeed_label] = kin.arm_angle_ikine_limit(Tr, self.qq_state, self.DH0, self.qq_min, self.qq_max)
        qr = self.kin.iterate_ikine(self.qq_state, Tr)

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
        beta = 1
        Xr = np.copy(self.xx_d + beta * self.Ex)
        Tr = np.eye(4)
        Rr = bf.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]

        # [qr, succeed_label] = kin.arm_angle_ikine_limit(Tr, self.qq_state, self.DH0, self.qq_min, self.qq_max)
        qr = self.kin.iterate_ikine(self.qq_state, Tr)

        return qr

#基于差分方程求解自适应导纳方程:还未调试成功
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

    def get_robot_parameter(self,DH_0, q_max, q_min,):
        #DH参数
        self.DH0 = np.copy(DH_0)
        self.theta0 = np.copy(DH_0[:, 0])
        self.alpha = np.copy(DH_0[:, 1])
        self.a = np.copy(DH_0[:, 2])
        self.d = np.copy(DH_0[:, 3])
        #关节极限
        self.qq_max = np.copy(q_max)
        self.qq_min = np.copy(q_min)
        #求取关节个数
        self.n = len(self.qq_max)

        #创建运动学类
        self.kin = kin.GeneralKinematic(DH_0)

    def get_current_joint(self, qq):
        self.qq_state = np.copy(qq)

    def get_current_force(self, F_t):
        self.base_f = self.force_end_to_base(F_t)
        #更新力误差
        self.Ef = self.base_f - self.ff_d
        self.E_f_list[0, :] = self.E_f_list[1, :]
        self.E_f_list[1, :] = self.E_f_list[2, :]
        self.E_f_list[2, :] = self.E_f_list[3, :]
        self.E_f_list[3, :] = self.Ef

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

    #基于双曲变换求取积分自适应导纳方程,单自由度方程
    def int_adp_imc_diff_eq(self, m, b, k, ki, T, ef, ex):
        '''
        :param md: 阻抗参数Md
        :param bd: 阻抗参数Bd
        :param kd: 阻抗参数Kd
        :param T: 控制周期T
        :param ef: 4个时刻的力偏差
        :param ex: 3个时刻的位置误差
        :return:
        '''
        w1 = 4 * m + 2 * b * T + k * T * T
        w2 = -12 * m - 2 * b * T + k * T * T
        w3 = 12 * m - 2 * b * T - k * T * T
        w4 = - 4 * m + 2 * b * T - k * T * T
        if (abs(m) < math.pow(10,-6)):
            ee = 0.0
        else:
           ee = (T**2/2.0*(((ki*T + 2)*ef[3] + (3*ki*T + 2)*ef[2] + \
                           (3*ki*T - 2)*ef[1]) + (ki*T - 2)*ef[0]) - \
                 w2*ex[2] + w3*ex[1] + w4*ex[0])/w1
        return ee

    def compute_imp_joint(self):
        # print "位置误差：", np.round(self.E_x_list, 3)
        # print "力误差：", np.round(self.E_f_list, 3)
        #计算末端位置偏差
        E = np.zeros(6)
        for i in range(6):
            if (abs(self.M[i]) > math.pow(10, -6)):
                E[i] = self.int_adp_imc_diff_eq(self.M[i],
                                              self.B[i],
                                              self.K[i],
                                              self.I[i],
                                              self.T,
                                              self.E_f_list[:, i],
                                              self.E_x_list[:, i])
        self.E_x_list[0, :] = self.E_x_list[1, :]
        self.E_x_list[1, :] = self.E_x_list[2, :]
        self.E_x_list[2, :] = E

        #计算参考位置
        print "误差修正项：", np.round(E, 6)
        beta = 1
        Xr = np.copy(self.xx_d - beta*E)
        Tr = np.eye(4)
        Rr = self.kin.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]

        qr = self.kin.iterate_ikine(self.qq_state, Tr, efs=pow(10, -8))

        return qr

#常规导纳控制
class IMPController_diff(object):
    #**定义属性**#
    #阻抗参数
    M = np.zeros(6)
    B = np.zeros(6)
    K = np.zeros(6)

    def __init__(self):
        #位置、力误差
        self.E_f_list = np.zeros([3, 6])
        self.E_x_list = np.zeros([2, 6])
        self.qq_state = np.zeros(7)

        #周期
        self.T = 0.01

    #**定义方法**#
    #获取控制周期
    def get_period(self, T):
        self.T = np.copy(T)

    #获取阻抗参数
    def get_imp_parameter(self,Md,Bd,Kd,Ki):
        self.M = np.copy(Md)
        self.B = np.copy(Bd)
        self.K = np.copy(Kd)

    def get_robot_parameter(self,DH_0, q_max, q_min,):
        #DH参数
        self.DH0 = np.copy(DH_0)
        self.theta0 = np.copy(DH_0[:, 0])
        self.alpha = np.copy(DH_0[:, 1])
        self.a = np.copy(DH_0[:, 2])
        self.d = np.copy(DH_0[:, 3])
        #关节极限
        self.qq_max = np.copy(q_max)
        self.qq_min = np.copy(q_min)
        #求取关节个数
        self.n = len(self.qq_max)

        #创建运动学类
        self.kin = kin.GeneralKinematic(DH_0)

    def get_current_joint(self, qq):
        self.qq_state = np.copy(qq)
        self.xx_state = self.kin.fkine_euler(self.qq_state)

        #更新位置误差
        self.Ex = self.xx_state - self.xx_d
        self.E_x_list[0, :] = self.E_x_list[1, :]
        self.E_x_list[1, :] = self.Ex

    def get_current_force(self, F_t):
        self.base_f = self.force_end_to_base(F_t)
        #更新力误差
        self.Ef = self.base_f - self.ff_d
        self.E_f_list[0, :] = self.E_f_list[1, :]
        self.E_f_list[1, :] = self.E_f_list[2, :]
        self.E_f_list[2, :] = self.Ef

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

    #基于双曲变换求取积分自适应导纳方程,单自由度方程
    def int_adp_imc_diff_eq(self, m, b, k, T, ef, ex):
        '''
        :param md: 阻抗参数Md
        :param bd: 阻抗参数Bd
        :param kd: 阻抗参数Kd
        :param T: 控制周期T
        :param ef: 3个时刻的力偏差
        :param ex: 2个时刻的位置误差
        :return:
        '''
        w1 = 4 * m + 2 * b * T + k * T * T
        w2 = -8 * m + 2*k * T * T
        w3 = 4 * m - 2 * b * T + k * T * T
        if (abs(m) < math.pow(10,-6)):
            ee = 0.0
        else:
            ee = (T ** 2 * ef[2] + 2 * T ** 2 * ef[1] + T ** 2 * ef[0] \
                  - w2 * ex[1] - w3 * ex[0]) / w1
        return ee

    def compute_imp_joint(self):
        # print "位置误差：", np.round(self.E_x_list, 3)
        # print "力误差：", np.round(self.E_f_list, 3)
        #计算末端位置偏差
        E = np.zeros(6)
        for i in range(6):
            if (abs(self.M[i]) > math.pow(10, -6)):
                E[i] = self.int_adp_imc_diff_eq(self.M[i],
                                              self.B[i],
                                              self.K[i],
                                              self.T,
                                              self.E_f_list[:, i],
                                              self.E_x_list[:, i])

        #计算参考位置
        print "误差修正项：", np.round(E, 6)
        beta = 1
        Xr = np.copy(self.xx_d + beta*E)
        Tr = np.eye(4)
        Rr = self.kin.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]

        qr = self.kin.iterate_ikine(self.qq_state, Tr, efs=pow(10, -8))

        return qr

#常规恒力控制:直接修正期望位置
class CIMPController_diff(object):
    #**定义属性**#
    #阻抗参数
    M = np.zeros(6)
    B = np.zeros(6)
    K = np.zeros(6)
    I = np.zeros(6)
    omega = np.zeros(6)

    def __init__(self):
        #位置、力误差
        self.E_f_list = np.zeros([3, 6])
        self.E_x_list = np.zeros([2, 6])
        self.qq_state = np.zeros(7)

        #周期
        self.T = 0.01

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

    def get_robot_parameter(self,DH_0, q_max, q_min,):
        #DH参数
        self.DH0 = np.copy(DH_0)
        self.theta0 = np.copy(DH_0[:, 0])
        self.alpha = np.copy(DH_0[:, 1])
        self.a = np.copy(DH_0[:, 2])
        self.d = np.copy(DH_0[:, 3])
        #关节极限
        self.qq_max = np.copy(q_max)
        self.qq_min = np.copy(q_min)
        #求取关节个数
        self.n = len(self.qq_max)

        #创建运动学类
        self.kin = kin.GeneralKinematic(DH_0)

    def get_current_joint(self, qq):
        self.qq_state = np.copy(qq)
        self.xx_state = self.kin.fkine_euler(self.qq_state)

        #更新位置误差
        self.Ex = self.xx_state - self.xx_d
        self.E_x_list[0, :] = self.E_x_list[1, :]
        self.E_x_list[1, :] = self.Ex

    def get_current_force(self, F_t):
        self.base_f = self.force_end_to_base(F_t)
        #更新力误差
        self.Ef = self.base_f - self.ff_d
        #加入修正项
        for i in range(6):
            if(abs(self.M[i]) > math.pow(10,-6)):
                self.omega[i] = self.omega[i] + self.I[i]*self.Ef[i]/self.B[i]

        self.E_f_list[0, :] = self.E_f_list[1, :]
        self.E_f_list[1, :] = self.E_f_list[2, :]
        self.E_f_list[2, :] = self.Ef

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

    #基于双曲变换求取积分自适应导纳方程,单自由度方程
    def int_adp_imc_diff_eq(self, m, b, k, T, ef, ex):
        '''
        :param md: 阻抗参数Md
        :param bd: 阻抗参数Bd
        :param kd: 阻抗参数Kd
        :param T: 控制周期T
        :param ef: 3个时刻的力偏差
        :param ex: 2个时刻的位置误差
        :return:
        '''
        w1 = 4 * m + 2 * b * T + k * T * T
        w2 = -8 * m + 2*k * T * T
        w3 = 4 * m - 2 * b * T + k * T * T
        if (abs(m) < math.pow(10,-6)):
            ee = 0.0
        else:
            ee = (T ** 2 * ef[2] + 2 * T ** 2 * ef[1] + T ** 2 * ef[0] \
                  - w2 * ex[1] - w3 * ex[0]) / w1
        return ee

    def compute_imp_joint(self):
        # print "位置误差：", np.round(self.E_x_list, 3)
        # print "力误差：", np.round(self.E_f_list, 3)
        #计算末端位置偏差
        E = np.zeros(6)
        for i in range(6):
            if (abs(self.M[i]) > math.pow(10, -6)):
                E[i] = self.int_adp_imc_diff_eq(self.M[i],
                                              self.B[i],
                                              self.K[i],
                                              self.T,
                                              self.E_f_list[:, i],
                                              self.E_x_list[:, i])

        #计算参考位置
        print "误差修正项：", np.round(E, 6)
        print "力误差修正项：",np.round(self.omega, 6)
        beta = 1
        Xr = np.copy(self.xx_d + beta*E + self.omega)
        Tr = np.eye(4)
        Rr = self.kin.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]

        qr = self.kin.iterate_ikine(self.qq_state, Tr, efs=pow(10, -8))

        return qr

#常规恒力控制:直接修速度项
class CIMPController_iter(object):
    #**定义属性**#
    #阻抗参数
    M = np.zeros(6)
    B = np.zeros(6)
    I = np.zeros(6)

    #构造函数
    def __init__(self):
        # 位置、力误差
        self.Ex = np.zeros(6)
        self.Ef = np.zeros(6)
        self.Ex_d = np.zeros(6)
        self.omega = np.zeros(6)
        # 周期
        self.T = 0.01
        self.first_flag = True

    #**定义方法**#
    #获取控制周期
    def get_period(self, T):
        self.T = np.copy(T)

    #获取阻抗参数
    def get_imp_parameter(self,Md,Bd,Kd,Ki):
        self.M = np.copy(Md)
        self.B = np.copy(Bd)
        self.I = np.copy(Ki)

    def get_robot_parameter(self, DH_0, q_max, q_min,):
        #DH参数
        self.DH0 = np.copy(DH_0)
        self.theta0 = np.copy(DH_0[:, 0])
        self.alpha = np.copy(DH_0[:, 1])
        self.a = np.copy(DH_0[:, 2])
        self.d = np.copy(DH_0[:, 3])
        #关节极限
        self.qq_max = np.copy(q_max)
        self.qq_min = np.copy(q_min)
        #求取关节个数
        self.n = len(self.qq_max)

        #创建运动学类
        self.kin = kin.GeneralKinematic(DH_0)

    def get_current_joint(self, qq):
        self.qq_state = np.copy(qq)

        #正运动学
        xx = self.kin.fkine_euler(qq)
        if (not self.first_flag):
            self.Ex_d = -(xx - self.xx_state)/self.T

        #正运动学
        self.xx_state = xx

        #正运动学,求取当前末端位置
        #self.Ex = self.xx_state - self.xx_d

    def get_current_force(self, F_t):
        #转换到基坐标系
        self.base_f = self.force_end_to_base(F_t)

        #更新力误差
        self.Ef = self.base_f - self.ff_d

    def get_expect_joint(self, qd):
        Xd = kin.fkine_euler(self.DH0, qd)
        self.xx_d = np.copy(Xd)

    def get_expect_pos(self, Xd):
        self.xx_d = np.copy(Xd)

    def get_expect_force(self,Fd):
        self.ff_d = np.copy(Fd)

    def force_end_to_base(self, F):
        base_F = np.zeros(6)

        Te = kin.fkine(self.theta0 + self.qq_state, self.alpha, self.a, self.d)
        Re = Te[0:3, 0:3]
        base_F[0:3] = np.dot(Re, F[0:3])
        base_F[3:6] = np.dot(Re, F[3:6])
        return base_F

    def int_adp_iter_solve(self, m, b, ki, T, ef, omega, ex, ex_d):
        #求取迭代项
        omega_k = omega + ki*ef/b

        # 求当前时刻速度
        x_dd = (ef - b * (ex_d + omega_k)) / m
        exk_d = ex_d + x_dd * T
        exk = ex + exk_d * T + x_dd * (T ** 2) / 2
        # print "积分项：",efk_i
        return [exk, exk_d, omega_k]

    def compute_imp_joint(self):
        #第一次运行
        self.first_flag = False
        #print "位置误差：", np.round(self.Ex, 3)
        #print "力误差：", np.round(self.Ef, 3)
        #计算末端位置偏差
        E = np.zeros(6)
        for i in range(6):
            if(self.M[i] > math.pow(10, -6)):
                [E[i], xv, self.omega[i]] = self.int_adp_iter_solve(
                    self.M[i], self.B[i],self.I[i],
                    self.T, self.Ef[i], self.omega[i],
                    self.Ex[i], self.Ex_d[i])
            else:
                E[i] = 0
            self.Ex[i] = E[i]

        #计算参考位置
        print "误差修正项：", np.round(E, 3)
        beta = 1
        Xr = np.copy(self.xx_d - beta*E)
        Tr = np.eye(4)
        Rr = bf.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]

        #[qr, flag] = kin.arm_angle_ikine_limit(Tr, self.qq_state,
        #                        self.DH0, self.qq_min, self.qq_max)
        qr = kin.iterate_ikine(self.DH0, self.qq_state, Tr, efs=pow(10, -12), i_max=1000)
        #print "qr:", qr

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

def main():
    #创建阻抗
    imp = IIMPController_iter()
    #创建阻抗参数调节器
    imp_param = IMPParamater()
    #输入阻抗参数
    M = np.zeros(6)
    M[2] = 2
    dM = -1
    B = np.zeros(6)
    B[2] = 400
    dB = -200
    K = np.zeros(6)
    K[2] = 0
    dK = 0
    I = np.zeros(6)
    I[2] = 0
    dI = 20

    a = 50
    b = 600

    imp_param.get_init_imp_paramter(M[2], B[2], K[2], I[2])
    imp_param.get_diff_imp_paramter(dM, dB, dK, dI)
    imp_param.get_function_paramter(a, b)

    #控制周期
    T = 0.01
    imp.get_period(T)

    #输入DH参数
    imp.get_robot_parameter(rp.DHfa_armc, rp.q_max_armc, rp.q_min_armc)

    num_imp = 1001
    num_init = 600
    num = num_init + num_imp
    k = 10

    # 环境刚度参数
    ke = 27000

    '''
    多次实验表明，需要解决两个问题
    (1)从初始位置到接触的瞬间冲击问题：超调量问题
    (2)接触后,环境曲面变化的适应问题：快速响应性
    '''

    #建立期望轨迹
    ld = 0.1
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
    le = 0.040
    Xe = np.copy(Xd)
    Xe_array = np.zeros([num, 6])
    for i in range(num):
        if(i<num_init):
            Xe_array[i, :] = Xe
            Xe_array[i, 0] = Xe[0] + ld * np.sin(0.5*np.pi / k * tt[0])
            Xe_array[i, 2] = Xe[2] - le * np.sin(2*np.pi / k * tt[0]) - 0.005
        else:
            Xe_array[i, :] = Xe
            Xe_array[i, 0] = Xe[0] + ld * np.sin(0.5*np.pi/k * tt[i-num_init])
            Xe_array[i, 2] = Xe[2] - le * np.sin(2*np.pi/k * tt[i-num_init]) - 0.005

    #建立期望力
    lf = 0.0
    Fd = np.array([0, 0, -15.0, 0, 0.0, 0])
    Fd_array = np.zeros([num, 6])
    for i in range(num):
        if(i<num_init):
            Fd_array[i, :] = Fd
            Fd_array[i, 2] = Fd[2] + lf * np.sin(2 * np.pi * tt[0])
        else:
            Fd_array[i, :] = Fd
            Fd_array[i, 2] = Fd[2] + lf * np.sin(2*np.pi * tt[i-num_init])

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
        [M[2], B[2], K[2], I[2]] = imp_param.out_expect_imp_paramter(i)
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
    main()