#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于本人开发的积分自适应阻抗相关函数,采用差分方程求解
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
from RobotParameter import DH0_armc

#================积分自适应阻抗方程参数MBKKi=================#
def get_imc_parameter():
    '''
    :return:
    '''
    #阻抗二阶方程系数
    Md = np.array([1,1,1,0,0,0])
    Bd = np.array([300,300,300,0,0,0])
    #Kd = np.array([0, 0, 0, 0, 0, 0])               #对应试教
    Kd = np.array([1000, 1000, 1000, 0, 0, 0])

    #积分参数ki<bd/md,一般取bd/md比值50分之1
    Ki = np.array([0, 0, 0, 0, 0, 0])  # 对应试教

    return [Md,Bd,Kd,Ki]

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

    #使用python引用机制,更新计算出的修正偏差
    ex[2] = ex[1]
    ex[1] = ex[0]
    ex[0] = ee
#The function end

#================阻抗控制外环，计算末端六维误差=================#
def get_current_error(Md,Bd,Kd,Ki,T,Ef,Ex):
    '''
    :param Md:惯性参数对角线
    :param Bd:阻尼参数对角线
    :param Kd:刚度参数对角线
    :param Ki:积分参数对角线
    :param T:
    :param Ef:上4个时刻末端力
    :param Ex:上3个时刻末端位置误差
    :return:
    '''

    ##建立关节当前3个时刻误差
    m = len(Ef[0,:])
    for i in range(m):
       int_adp_imc_diff_eq(Md[i],Bd[i],Kd[i],Ki[i],T,Ef[:,i],Ex[:,i])
# The function end

#================阻抗控制外环，计算关节角=================#
def get_current_joint(Xd,Ex,qq_init,DH_0 = DH0_armc):
    '''
    :param Xd:期望位置
    :param Ex:力偏偏差修正值
    :param qq_init:初始关节角
    :return:qq当前时刻关节角
    '''

    ##姿态偏差类似姿态速度，需要转换为欧拉角偏差
    theta0 = DH_0[:, 0];alpha = DH_0[:, 1];a = DH_0[:, 2];d = DH_0[:, 3]
    Te = kin.fkine(theta0 + qq_init,alpha,a,d)
    phi = bf.rot2euler_zyx(Te[0:3,0:3])
    inv_J_zyx= nla.inv(bf.J_euler_zyx(phi))

    #建立局部变量,避免改变传入参数
    E = np.copy(Ex)
    E[3:6] = np.dot(inv_J_zyx,E[3:6])
    Xr = Xd + E

    #将末端位姿转换为齐次矩阵
    Te = np.ones([4,4])
    Re = bf.euler_zyx2rot(Xr[3:6])
    Te[0:3,0:3] = Re
    Te[0:3,3] = Xr[0:3]

    #调用运动学方程求取关节角
    qq = kin.iterate_ikine(DH0_armc,qq_init,Te)
    return qq

#================期望位置获取=================#
def get_init_expect_pos():
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

#================将六维力转换到基座标系=================#
def force_end_to_base(qq,F,DH0 = DH0_armc):
    '''
    :param qq:
    :param F:
    :param DH0:
    :return:base_F
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
