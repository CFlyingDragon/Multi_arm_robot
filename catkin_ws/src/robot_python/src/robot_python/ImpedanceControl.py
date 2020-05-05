#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于阻抗基本函数
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.11.21
import numpy as np
from math import pi
import numpy.linalg as nla
import sys

#自定义文件
import BaseFunction as bf
import Kinematics as kin
from RobotParameter import DHf_armc

#================将六维力转换到机器人基座标系=================#
def force_end_to_base(qq,F,DH0 = DHf_armc):
    '''
    :param qq:当前时刻关节角
    :param F:当前时刻末端六维力
    :param DH0:机械臂DH参数
    :return:base_F相对与基座标系的力
    '''
    base_F = np.zeros(6)
    theta0 = DH0[:,0]
    alpha = DH0[:,1]
    a = DH0[:,2]
    d = DH0[:,3]

    Te = kin.fkine(theta0 + qq, alpha, a, d)
    Re = Te[0:3,0:3]
    base_F[0:3] = np.dot(Re, F[0:3])
    base_F[3:6] = np.dot(Re, F[3:6])
    return base_F

#================机器人正运动学，关节角转化为六自由度位姿=================#
def joint_to_pos(qq,DH0 = DHf_armc):
    '''
    :param qq:关节角
    :param DH0:
    :return:X末端位姿
    '''
    X = np.zeros(6)
    theta0 = DH0[:, 0]
    alpha = DH0[:, 1]
    a = DH0[:, 2]
    d = DH0[:, 3]

    Te = kin.fkine(theta0 + qq, alpha, a, d)
    phi = bf.rot2euler_zyx(Te[0:3,0:3])
    X[0:3] = Te[3,0:3]
    X[0:3] = phi
    return X

#================阻抗控制外环，计算关节角=================#
def get_current_joint(Xd,Ex,qq_guess,omega_k = np.zeros(6),DH_0 = DHf_armc):
    '''
    :param Xd:期望位置
    :param E:力偏偏差修正值
    :param qq_guess:关节角猜测值，用于求解
    :param omega_k:上一时刻自适应值
    :return:qq当前时刻关节角
    '''

    ##姿态偏差类似姿态速度，需要转换为欧拉角偏差
    E = np.copy(Ex)
    theta0 = DH_0[:, 0];alpha = DH_0[:, 1];a = DH_0[:, 2];d = DH_0[:, 3]

    Te = kin.fkine(theta0 + qq_guess,alpha,a,d)
    phi = bf.rot2euler_zyx(Te[0:3,0:3])
    inv_J_zyx= nla.inv(bf.J_euler_zyx(phi))
    E[3:6] = np.dot(inv_J_zyx,E[3:6])

    #加入自适应项，采用方法二，相当于修改期望位置
    Xr = Xd + omega_k + E

    #将末端位姿转换为齐次矩阵
    Te = np.ones([4,4])
    Re = bf.euler_zyx2rot(Xr[3:6])
    Te[0:3,0:3] = Re
    Te[0:3,3] = Xr[0:3]

    #调用运动学方程求取关节角
    qq = kin.iterate_ikine(DH_0,qq_guess,Te)
    return qq

#================积分自适应阻抗控制外环，计算Xr=================#
def imp_get_pos(M,B,K,I,F,Ef,Ex,T):
    pass


#================阻抗控制外环，计算关节角,输入形式为Xr=================#
def get_control_joint(Xr,qq_guess,q_max,q_min,d_h0,DH_0= DHf_armc):
    '''
    :param Xr:参考关节角
    :param qq_guess:关节角猜测值，用于求解
    :return:qr当前时刻关节角命令
    '''
    #将末端位姿转换为齐次矩阵
    Te = np.eye([4,4])
    Re = bf.euler_zyx2rot(Xr[3:6])
    Te[0:3,0:3] = Re
    Te[0:3,3] = Xr[0:3]

    #调用运动学方程求取关节角
    [qr, dh] = kin.w_ikine(DH_0,qq_guess,q_max,q_min,d_h0,Te)
    return [qr,dh]

#================积分自适应阻抗差分方程=================#
def int_adp_imc_diff_eq(md,bd,kd,ki,T,ef,ex):
    '''
    :param md: 阻抗参数Md
    :param bd: 阻抗参数Bd
    :param kd: 阻抗参数Kd
    :param T: 控制周期T
    :param ef: 4个时刻的力偏差
    :param ex: 3个时刻的位置误差
    :return:
    '''
    w1 = 4*md + 2*bd*T + kd*T*T
    w2 = 2*kd*T*T - 8*md
    w3 = kd*T*T + 4*md - 2*bd*T
    if(w1==0):
        ee = 0.0
    else:
        ee = 1/w3*(w1*ex[2]+(w2-w1)*ex[1]+(w3-w2)*ex[0])\
             - T*T/(2*w3)*((T*ki+2)*ef[3]+(3*T*ki+2)*ef[2]\
                           + (3*T*ki-2)*ef[1]+(T*ki-2)*ef[0])  #\转行符后不可注释

    #更新偏差
    ex_next = np.zeros(3)
    ex_next[0] = ex[1]
    ex_next[1] = ex[2]
    ex_next[2] = ee
    return ex_next

#================创建一个适用于积分自适应的类=================#
class IMPController(object):
    #**定义属性**#
    #阻抗参数
    M = np.zeros(6)
    B = np.zeros(6)
    K = np.zeros(6)
    I = np.zeros(6)
    #位置、力误差
    Ex = list(np.zeros([3,6]))
    Ef = list(np.zeros([4,6]))
    #周期
    T = 0.01

    #**定义方法**#
    #获取控制周期
    def get_period(self,T):
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
        # 优化函数
        self.dh = np.zeros(self.n)

    def get_current_joint(self,qq):
        self.qq_state = np.copy(qq)
        #正运动学,求取当前末端位置
        joint_to_pos()
        #更新末端位置误差----?
        ee = self.xx_state - self.xx_d
        self.Ex.append(ee)
        del self.Ex[0]

    def get_current_force(self,F_t):
        base_f = np.zeros(6)

        Te = kin.fkine(self.theta0 + self.qq_state, self.alpha, self.a, self.d)
        Re = Te[0:3, 0:3]
        base_f[0:3] = np.dot(Re, F_t[0:3])
        base_f[3:6] = np.dot(Re, F_t[3:6])
        self.base_f = np.copy(base_f)
        #更新力误差
        ff = self.base_f - self.ff_d
        self.Ef.append(ff)
        del self.Ef[0]

    def get_expect_pos(self,Xd):
        self.xx_d = np.copy(Xd)

    def get_expect_force(self,Fd):
        self.ff_d = np.copy(Fd)

    def joint_to_pos(self):
        X = np.zeros(6)
        Te = kin.fkine(self.theta0 + self.qq_state, self.alpha,self.a, self.d)
        phi = bf.rot2euler_zyx(Te[0:3, 0:3])
        X[0:3] = Te[3, 0:3]
        X[0:3] = phi
        self.xx_state = np.copy(X)

    def int_adp_imc_diff_eq(self,md,bd,kd,ki,T,ef,ex):
        w1 = 4 * md + 2 * bd * T + kd * T * T
        w2 = 2 * kd * T * T - 8 * md
        w3 = kd * T * T + 4 * md - 2 * bd * T
        #求取偏差
        ee = 0.0
        if (w1 == 0):
            ee = 0.0
        else:
            ee = 1 / w3 * (w1 * ex[2] + (w2 - w1) * ex[1] + (w3 - w2) * ex[0]) \
                 - T * T / (2 * w3) * ((T * ki + 2) * ef[3] + (3 * T * ki + 2) * ef[2] \
                                + (3 * T * ki - 2) * ef[1] + (T * ki - 2) * ef[0])
        # 更新偏差
        return ee

    def compute_imp_joint(self):
        #计算末端位置偏差
        E = np.zeros(6)
        for i in range(6):
            ef = np.array(self.Ef[0])
            ex = np.array(self.Ex[0])
            E[i] = self.int_adp_imc_diff_eq(self.M[i],self.B[i],
                                            self.K[i],self.I[i],
                                            self.T,ef,ex)
        #计算参考位置
        Xr = np.copy(self.xx_d + E)
        Tr = np.eye(4)
        Rr = bf.euler_zyx2rot(Xr[3:6])
        Tr[0:3, 0:3] = Rr
        Tr[0:3, 3] = Xr[0:3]

        [qr, dh] = kin.w_ikine(self.DH0, self.qq_state, self.qq_max,
                               self.qq_min, self.dh, Tr)
        self.dh = np.copy(dh)
        return qr







