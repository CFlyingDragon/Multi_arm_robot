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

#自定义文件
import BaseFunction as bf
import Kinematics as kin

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
        self.Ex = np.zeros(6)
        self.Ef = np.zeros(6)
        self.Ex_d = np.zeros(6)
        self.Ef_i = np.zeros(6)
        # 周期
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

    def get_expect_force(self, Fd):
        self.ff_d = np.copy(Fd)

    def force_end_to_base(self, F):
        base_F = np.zeros(6)

        Te = kin.fkine(self.theta0 + self.qq_state, self.alpha, self.a, self.d)
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
        exk = ex + exk_d * T #+ ex_dd * (T ** 2) / 2

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
        beta = 0.1
        Xr = np.copy(self.xx_d - beta*self.Ex)
        Tr = np.eye(4)
        Rr = bf.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]

        qr = kin.iterate_ikine(self.DH0, self.qq_state, Tr, efs=pow(10, -12), i_max=1000)

        return qr

#基于差分方程求解自适应导纳方程
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
        self.xx_state = self.kin.fkine_euler(self.qq_state)

        #更新位置误差
        #self.Ex = self.xx_state - self.xx_d


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
