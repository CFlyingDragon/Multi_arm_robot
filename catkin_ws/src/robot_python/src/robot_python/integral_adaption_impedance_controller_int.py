#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于本人开发的积分自适应阻抗相关函数,采用一阶积分方程求解
#本文采用了Python引用机制
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.1.12
import numpy as np
from math import pi
import numpy.linalg as nla
import sys

#自定义文件
import BaseFunction as bf
import Kinematics as kin
from RobotParameter import DHf_armc

#================积分自适应阻抗方程参数MBKKi=================#
def get_imc_parameter():
    '''
    :return:
    '''
    #阻抗二阶方程系数
    Md = np.array([1,0,0,0,0,0])
    Bd = np.array([300,300,300,0,0,0])
    #Kd = np.array([0, 0, 0, 0, 0, 0])               #对应试教
    Kd = np.array([1000, 1000, 1000, 0, 0, 0])

    #积分参数ki<bd/md,一般取bd/md比值50分之1
    Ki = np.array([0, 0, 0, 0, 0, 0])  # 对应试教

    return [Md,Bd,Kd,Ki]

#================采用一阶积分求取力误差=================#
def force_integral(Ki,delta_F,omega):
    '''
    :param Ki:
    :param delta_F:
    :param omega:
    :return:
    '''
    #通过引用改变值
    omega_c = np.zeros(6)
    for i in range(6):
        omega_c[i] = omega[i] + Ki[i]*delta_F[i]
    return omega_c

#================积分自适应阻抗积分方程=================#
def int_adp_imc_int_eq(T,Xd,Xvd,Xad,delta_F,Xr,Xvr,omega):
    '''
    :param T:
    :param Xd:
    :param Xvd:
    :param Xad:
    :param delta_F:
    :param Xr: 参考轨迹
    :param Xvr:参考轨迹速度
    :param omega:力误差积分项
    :return:
    '''
    #获取姿态角到欧拉角速度转化矩阵
    inv_J_zyx = nla.inv(bf.J_euler_zyx(Xr[3:6]))

    #获取阻抗参数
    [Md, Bd, Kd, Ki] = get_imc_parameter()

    #求取力误差积分项
    omega_c = force_integral(Ki, delta_F, omega)

    #求取参考轨迹
    Xar = np.copy(Xad)
    for i in range(6):
        if(Md[i]< pow(10,-6)):
            Xar[i] = Xad[i]
        else:
            Xar[i] = Xad[i] + 1/Md[i]*(delta_F[i] + omega_c[i] - Bd[i]*(Xvr[i] - Xvd[i]) - Kd[i]*(Xr[i] - Xd[i]))
    print "参考加速度Xar:%s" % Xvd
    Xvr_c = Xvr + Xar*T
    Xvr_w = np.copy(Xvr)
    Xvr_w[3:6] = np.dot(inv_J_zyx, Xvr_w[3:6])
    Xr_c = Xr + Xvr_w*T
    print "参考速度Xv:%s" % Xvr
    print "参考位置Xr:%s" % Xr
    return [Xr_c,Xvr_c,omega_c]

#================期望位置获取=================#
def get_init_expect_pos(DH_0 = DHf_armc):
    '''
    :return:
    '''
    #DH参数
    theta0 = DH_0[:, 0]
    alpha = DH_0[:, 1]
    a = DH_0[:, 2]
    d = DH_0[:, 3]

    #期望末端位姿
    Xd = np.zeros(6)
    qr_init = np.array([0, 30, 0, 60, 0, 30, 0]) * (pi / 180)
    Te = kin.fkine(theta0 + qr_init, alpha, a, d)
    Xd[0:3] = Te[0:3,3]
    Xd[3:6] = np.array([0,pi/2,0])
    Xvd = np.zeros(6)
    Xad = np.zeros(6)
    return [Xd,Xvd,Xad,qr_init]

