#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于多机械臂规划程序
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020/4/16
import numpy as np
import BaseFunction as bf
import Robots
import PathPlan as pap

#=================工件轨迹规划-直线规划=================#
def workpiece_moving_track_line(Xb,Xe):
    '''
    :return:
    '''
    z = np.zeros(6)
    #轨迹点
    Xw = np.zeros([251, 6])
    Xvw = np.zeros([251, 6])
    Xaw = np.zeros([251, 6])
    #采用5次多项式平滑轨迹
    t = 10
    T = 0.1
    [Xw1, Xvw1, Xaw1] = bf.interp5rdPoly(Xb,z,z,Xe,z,z,t,T)

    ##全段轨迹
    Xw[0:101, :] = Xw1
    Xvw[0:101, :] = Xvw1
    Xaw[0:101, :] = Xaw1
    for i in range(50):
        Xw[101+i, :] = Xw1[-1,:]
        Xvw[101+i, :] = Xvw1[-1,:]
        Xaw[101+i, :] = Xaw1[-1,:]
    for i in range(100):
        Xw[151+i, :] = Xw1[100-i,:]
        Xvw[151+i, :] = Xvw1[100-i,:]
        Xaw[151+i, :] = Xaw1[100-i,:]
    return Xw

#=================工具坐标系在基座标系中的位置=================#
def arms_end_track(Xb,C,Xw):
    '''
    :param Xb:
    :param C:
    :param Xw:
    :return:
    '''
    #初始值
    k = Xw[:,0] #规划点个数
    m = len(Xb[:,0]) #机械臂个数
    Xe = np.zeros([k,m,6])
    #工具末端在基座标系下的表示
    for i in range(k):
        Xe[i,:,:] = Robots.tool_to_base(Xb,C,Xw[i,:])
    return Xe

#=================多机械臂关节角规划=================#
def ur5s_joint_plan(Xw,Xb,C,qr_init,DH_0, q_max, q_min):
	'''
	:param Xw:
	:param Xb:
	:param C:
	:param qr_init:
	:param DH_0:
	:param q_max:
	:param q_min:
	:return:
	'''
	##定义关节变量
	T = 0.1 #操作空间采样频率

	##获取机械臂工具坐标系的位置
	Xe = arms_end_track(Xb, C, Xw)
	##带入加权最小范数法规划求取关节角
	[qq1,qv1,qa1] = pap.multipoint_plan_position_w(qr_init[0,:], Xe[:,0,:],
												   T, DH_0, q_max, q_min)
	[qq2,qv2,qa2] = pap.multipoint_plan_position_w(qr_init[1,:], Xe[:,1,:],
												   T, DH_0, q_max, q_min)
	return [qq1,qq2]

def armc_joint_plan(Xw,Xb,C,qr_init,DH_0, q_max, q_min):
    '''
    :return:
    '''
    ##定义关节变量
    T = 0.1 #操作空间采样频率

    ##获取机械臂工具坐标系的位置
    Xe = arms_end_track(Xb, C, Xw)
    ##带入加权最小范数法规划求取关节角
    [qq3,qv3,qa3] = pap.multipoint_plan_position_w(qr_init[0,:], Xe[:,0,:],
                                                   T, DH_0, q_max, q_min)
    return qq3


