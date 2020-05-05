#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于规划中的末端轨迹求取,默认等时间取样
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.10.1
import numpy as np
import math
from math import pi

#自定义函数
import Kinematics as kin
from RobotParameter import DH0_armc
import BaseFunction as bf

#=================自制机械臂armc圆轨迹末端笛卡尔空间位置点=================#
def circlePoint_armc(rc):
	'''
		本函数用于自制机械臂armc特定位置圆规划
		input:rc规划圆的半径
		output:[q_init,X0_e,T] 规划初始点关节位置、末端位姿点，取样周期
	'''
	#获得规划机械臂armc的DH参数表
	DH_0 = DH0_armc
	theta0 = DH_0[:,0];alpha = DH_0[:,1];a = DH_0[:,2];d = DH_0[:,3]
	#关节个数
	n = len(theta0)
	
	#规划位置初始点
	qr_init = np.array([0,30, 0, 30, 0 , 30, 0])*(np.pi/180)
	T_init = kin.fkine(theta0 + qr_init,alpha,a,d)
	
	#建立规划空间坐标系(默认规划坐标系与初始位置7关节坐标系方向相同，原点在（rc,0,0))
	Tb_c = np.array([[1,0,0,rc],
					 [0,1,0,0 ],
					 [0,0,1,0 ],
					 [0,0,0,1]])
	T0_c = np.dot(T_init,Tb_c)
	
	#取样周期
	T = 0.1
	
	#规划空间建立圆坐标点,从z_c轴看，逆时针旋转360度
	nodeNum = 381
	t = np.linspace(0,T*(nodeNum-1),nodeNum)
	phi = np.zeros(nodeNum)
	##计算转动的角度
	#加速阶段,采用5次多项式插值   (平滑处理）
	phi_b0 = 0;phiv_b0 = 0;phia_b0 = 0;
	phi_bf = pi/18; phiv_bf = pi/18; phia_bf = 0;
	tf = 2; dt = T;
	[phi[0:21],v_,a_] = bf.interp5rdPoly1(phi_b0,phiv_b0,phia_b0,phi_bf,phiv_bf,phia_bf,tf,dt)
	#匀速阶段
	phi[21:360] = np.linspace(pi/18 + pi/180,35*pi/18 - pi/180,339)
	#减速速阶段,采用5次多项式插值
	phi_e0 = 35*pi/18;phiv_e0 = pi/18;phia_e0 = 0;
	phi_ef = 2*pi; phiv_ef = 0; phia_ef = 0;
	tf = 2; dt = T;
	[phi[360:381],v_,a_] = bf.interp5rdPoly1(phi_e0,phiv_e0,phia_e0,phi_ef,phiv_ef,phia_ef,tf,dt)
	
	#起始点偏置
	phi = phi + pi*np.ones(nodeNum)
	
	#末端圆在规划空间笛卡尔坐标系的齐次位置
	pc_e = np.ones([nodeNum,4])
	for i in range(nodeNum):
		pc_e[i,0] = rc*np.cos(phi[i])
		pc_e[i,1] = rc*np.sin(phi[i])
		pc_e[i,2] = 0
	
	#由姿态保持初始姿态不变，转为Euler_ZYX角
	phi_zyx = np.array([0, pi/2, 0])
	
	#求取末端规划点在位姿在0号坐标系下的表示
	X0_e = np.zeros([nodeNum,6])
	for i in range(nodeNum): 
		X0_e[i,0:3] = np.dot(T0_c,pc_e[i,:])[0:3]
		X0_e[i,3:6] = phi_zyx
		
	return [qr_init,X0_e,T]

#=================自制机械臂armc圆轨迹末端笛卡尔空间速度点=================#
def circleVelPoint_armc(rc):                                        #尚未规划完成
	'''
		本函数用于自制机械臂armc特定位置圆规划
		input:rc规划圆的半径
		output:[q_init,X0_e,T] 规划初始点关节位置、末端位姿点，取样周期
	'''
	#获得规划机械臂armc的DH参数表
	DH_0 = DH0_armc
	theta0 = DH_0[:,0];alpha = DH_0[:,1];a = DH_0[:,2];d = DH_0[:,3]
	#关节个数
	n = len(theta0)
	
	#规划位置初始点
	qr_init = np.array([0,-30, 0, 90, 0 , 30 , 0])*(np.pi/180)
	T_init = kin.fkine(theta0 + qr_init,alpha,a,d)
	
	#建立规划空间坐标系(默认规划坐标系与初始位置7关节坐标系方向相同，原点在（rc,0,0))
	Tb_c = np.array([[1,0,0,rc],
					 [0,1,0,0 ],
					 [0,0,1,0 ],
					 [0,0,0,1]])
	T0_c = np.dot(T_init,Tb_c)
	
	#取样周期
	T = 0.1
	
	#规划空间建立圆坐标点,从z_c轴看，逆时针旋转360度
	nodeNum = 381
	t = np.linspace(0,T*(nodeNum-1),nodeNum)
	phi = np.zeros(nodeNum)
	##计算转动的角度
	#加速阶段,采用5次多项式插值   (平滑处理）
	phi_b0 = 0;phiv_b0 = 0;phia_b0 = 0;
	phi_bf = pi/18; phiv_bf = pi/18; phia_bf = 0;
	tf = 2; dt = T;
	[phi[0:21],v_,a_] = bf.interp5rdPoly1(phi_b0,phiv_b0,phia_b0,phi_bf,phiv_bf,phia_bf,tf,dt)
	#匀速阶段
	phi[21:360] = np.linspace(pi/18 + pi/180,35*pi/18 - pi/180,339)
	#减速速阶段,采用5次多项式插值
	phi_e0 = 35*pi/18;phiv_e0 = pi/18;phia_e0 = 0;
	phi_ef = 2*pi; phiv_ef = 0; phia_ef = 0;
	tf = 2; dt = T;
	[phi[360:381],v_,a_] = bf.interp5rdPoly1(phi_e0,phiv_e0,phia_e0,phi_ef,phiv_ef,phia_ef,tf,dt)
	
	#起始点偏置
	phi = phi + pi*np.ones(nodeNum)
	
	#末端圆在规划空间笛卡尔坐标系的齐次位置
	pc_e = np.ones([4,nodeNum])
	for i in range(nodeNum):
		pc_e[0,i] = rc*np.cos(phi[i])
		pc_e[1,i] = rc*np.sin(phi[i])
		pc_e[2,i] = 0
	
	#由姿态保持初始姿态不变，转为Euler_ZYX角
	phi_zyx = np.array([0, pi/2, 0])
	
	#求取末端规划点在位姿在0号坐标系下的表示
	X0_e = np.zeros([nodeNum,6])
	for i in range(nodeNum): 
		X0_e[i,0:3] = np.dot(T0_c,pc_e[:,i])[0:3]
		X0_e[i,3:6] = phi_zyx
		
	return [qr_init,X0_e,T]

#=================自制机械臂armc直线轨迹末端笛卡尔空间位置点=================#
def multipointLine_armc(l):
	'''
		本函数用于自制机械臂armc特定位置直线规划
		input:l直线长度
		output:[q_init,X0_e,T] 规划初始点关节位置、末端位姿点,取样周期
	'''
	#获得规划机械臂armc的DH参数表
	DH_0 = DH0_armc
	theta0 = DH_0[:,0];alpha = DH_0[:,1];a = DH_0[:,2];d = DH_0[:,3]
	#关节个数
	n = len(theta0)
	
	#规划位置初始点
	qr_init = np.array([0,-30, 0, 90, 0 , 30 , 0])*(np.pi/180)
	T_init = kin.fkine(theta0 + qr_init,alpha,a,d)
	
	#取样周期
	T = 0.01
	
	#规划直线较简单，直接在0号坐标系先规划
	nodeNum = 100
	X0_e = np.zeros([6,nodeNum])
	
	#由姿态保持初始姿态不变，转为Euler_ZYX角
	phi_zyx = np.array([0, pi/2, 0])
	
	#直线起始点和末端点
	pe_b = T_init[0:3,3]
	pe_e = T_init[0:3,3] + np.array([l,0,0])
	
	#0号坐标系下的位姿
	for i in range(nodeNum): 
		X0_e[0:3,i] = pe_b + (pe_e - pe_b)*np.sin((pi/2)*(i/(1.0*(nodeNum - 1))))
		X0_e[3:6,i] = phi_zyx
		
	return [qr_init,X0_e,T]


