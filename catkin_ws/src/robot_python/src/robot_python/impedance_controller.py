#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用用于建立简单通用的阻抗控制器
#本文采用了Python引用机制
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.1.12
import numpy as np
from math import pi
import numpy.linalg as nla
from RobotParameter import DH0_armc
import sys

#自定义文件
import BaseFunction as bf
import Kinematics as kin
from RobotParameter import DHf_armc

# ================阻抗差分方程=================#
def imc_difference_equation(md, bd, kd, T, f, e):
    '''
    :param md: 阻抗参数Md
    :param bd: 阻抗参数Bd
    :param kd: 阻抗参数Kd
    :param T: 控制周期T
    :param f: 3个时刻的力
    :param e: 3个时刻的误差
    :return:e,最新三个时刻末端位置误差
    '''

    w1 = 4 * md + 2 * bd * T + kd * T * T
    w2 = 2 * kd * T * T - 8 * md
    w3 = kd * T * T + 4 * md - 2 * bd * T
    if (w1 == 0):
        ee = 0.0
    else:
        ee = T * T / w1 * (f[2] + 2 * f[1] + f[0]) - w2 / w1 * e[1] - w3 / w1 * e[0]
    current_e = np.zeros(2)
    current_e[0] = e[1]
    current_e[1] = ee
    return current_e


# ================阻抗控制外环，计算末端六维误差=================#
def get_current_error(Md, Bd, Kd, T, F, E):
    '''
    :param Md:惯性参数对角线
    :param Bd:阻尼参数对角线
    :param Kd:刚度参数对角线
    :param T:
    :param F:上三个时刻末端力
    :param E:上三个时刻末端位置误差
    :return:current_E，当前3个时刻误差
    '''

    ##建立关节当前3个时刻误差
    m = len(F[0, :])
    current_E = np.zeros([2, m])

    for i in range(m):
        current_E[:, i] = imc_difference_equation(Md[i], Bd[i], Kd[i], T, F[:, i], E[:, i])

    return current_E


#================阻抗差分方程参数MBK=================#
def get_imc_parameter():
    '''

    :return:
    '''

    Md = np.array([1,1,1,0,0,0])
    Bd = np.array([900,900,900,0,0,0])
    #Bd = np.array([90, 90, 90, 10, 10, 10])   #小阻尼
    #Kd = np.array([900000,900000,900000,0,0,0])    ##对应阻抗控制
    Kd = np.array([0, 0, 0, 0, 0, 0])               #对应试教
    #Kd = np.array([1000, 1000, 1000, 0, 0, 0])  # 小刚度度

    return [Md,Bd,Kd]

#================期望位置获取=================#
def get_expect_pos():
    '''

    :return:
    '''
    #DH参数
    theta0 = DH0_armc[:, 0]
    alpha = DH0_armc[:, 1]
    a = DH0_armc[:, 2]
    d = DH0_armc[:, 3]

    #期望末端位姿
    Xd = np.zeros(6)
    qr_init = np.array([0, 30, 0, 60, 0, 30, 0]) * (pi / 180)
    Te = kin.fkine(theta0 + qr_init, alpha, a, d)
    Xd[0:3] = Te[0:3,3]
    Xd[3:6] = np.array([0,pi/2,0])

    return [Xd,qr_init]

#================自适应项=================#
def force_adaption(K,Fd,F,omega_k):
    '''
    :param K:
    :param Fd:
    :param F:
    :param omega_k:
    :return:
    '''
    n = 6
    omega_kk = np.zeros(n)
    eta = np.array([0.1,0.1,0.1,0.1,0.1,0.1])
    for i in range(n):
        omega_kk[i] = omega_k[i] + eta[i]*(Fd[i]-F[i])/K[i]

    return omega_kk

#================阻抗控制外环，计算关节角=================#
def get_extend_current_joint(Xd,E,qq_init,omega_k,DH_0 = DH0_armc):
    '''
    :param Xd:期望位置
    :param E:力偏偏差修正值
    :param qq_init:初始关节角
    :param omega_k:上一时刻自适应值
    :return:qq当前时刻关节角
    '''

    ##姿态偏差类似姿态速度，需要转换为欧拉角偏差
    theta0 = DH_0[:, 0];alpha = DH_0[:, 1];a = DH_0[:, 2];d = DH_0[:, 3]
    Te = kin.fkine(theta0 + qq_init,alpha,a,d)
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
    psi_r = Xr[7]

    #调用运动学方程求取关节角
    qq = kin.aa_ikine(Te,psi_r,qq_init)

    return qq

#================扩展末端空间阻抗差分方程参数MBK=================#
def get_extend_imc_parameter():
    '''

    :return:
    '''

    Md = np.array([1,1,1,1,1,1,1])
    #Bd = np.array([900,900,900,0,0,0])
    Bd = np.array([90, 90, 90, 10, 10, 10, 10])   #小阻尼
    #Kd = np.array([900000,900000,900000,0,0,0])    ##对应阻抗控制
    #Kd = np.array([0, 0, 0, 0, 0, 0])               #对应试教
    Kd = np.array([1000, 1000, 1000, 100, 100, 100, 100])  # 小刚度度

    return [Md,Bd,Kd]