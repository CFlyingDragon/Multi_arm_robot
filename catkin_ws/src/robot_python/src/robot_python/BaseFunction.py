#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于规划中的末端轨迹求取,默认等时间取样
#程序员：陈永厅
#版权：哈尔滨工业大学BBb
#日期：初稿：2019.11.26

import numpy as np
import numpy.linalg as nla
import math
from math import pi

#====================叉乘操作矩阵=====================#
def cross_matrix(v):
	'''
		input:v三维向量
		output:3×3矩阵
	'''
	v_matrix = np.array([[0,    -v[2], v[1]],
						[v[2],   0,   -v[0]],
						[-v[1], v[0],  0]])
	return v_matrix

#=================1维行列向量相乘为矩阵=================#
def vector2matrix(v1,v2):
	'''
		一维行向量与一维列向量的乘积
		input:v1、v2向量
		output:返回矩阵
	'''
	m = len(v1)
	n = len(v2)
	V1 = np.zeros([m,1])
	V2 = np.zeros([1,n])
	V1[:,0] = v1
	V2[0,:] = v2
	M = np.dot(V1,V2)
	return M

#===============坐标姿态各种表示之间的转换===============#
#欧拉轴角到旋转矩阵rotate
def euler_axis2rot(n,theta):
	'''
		本函数将欧拉角转化为旋转矩阵
		input:n欧拉轴、theta转角
		output:R旋转矩阵
	'''
	n_m = cross_matrix(n)
	R = np.eye(3) + n_m*np.sin(theta) + np.dot(n_m,n_m)*(1 - np.cos(theta))
	return R

#ZYX欧拉角转变为旋转矩阵
def euler_zyx2rot(phi):
	'''
		ZYX欧拉角转变为旋转矩阵
		input:欧拉角
		output:旋转矩阵
	'''
	R = np.array([[np.cos(phi[0])*np.cos(phi[1]), np.cos(phi[0])*np.sin(phi[1])*np.sin(phi[2]) - np.sin(phi[0])*np.cos(phi[2]),
		np.cos(phi[0])*np.sin(phi[1])*np.cos(phi[2]) + np.sin(phi[0])*np.sin(phi[2])],
		[np.sin(phi[0])*np.cos(phi[1]), np.sin(phi[0])*np.sin(phi[1])*np.sin(phi[2]) + np.cos(phi[0])*np.cos(phi[2]),
		np.sin(phi[0])*np.sin(phi[1])*np.cos(phi[2]) - np.cos(phi[0])*np.sin(phi[2])],
		[-np.sin(phi[0]), np.cos(phi[1])*np.sin(phi[2]), np.cos(phi[1])*np.cos(phi[2])]])
	return R

#旋转矩阵转变为ZYX欧拉角
def  rot2euler_zyx(Re):
	'''
		ZYX欧拉角速度变为姿态角速度转化矩阵
		input:旋转矩阵
		output:欧拉角[alpha,beta,gamma]
	'''
	euler_zyx = np.zeros(3)
	if((np.abs(Re[2,1]) < pow(10,-12)) and(np.abs(Re[2,2]) < pow(10,-12))):
		if(Re[2,0] < 0):
			beta = pi/2
			alpha = np.arctan2(-Re[1,2],Re[1,1])
			gamma = 0
		else:
			beta = -pi/2
			alpha = -np.arctan2(-Re[1, 2], Re[1, 1])
			gamma = 0
	else:
		p_beta = math.asin(-Re[2,0])
		cb = np.cos(p_beta)
		alpha = math.atan2(Re[1,0]*cb,Re[0,0]*cb)
		gamma = math.atan2(Re[2,1]*cb,Re[2,2]*cb)
		if((math.sin(gamma)*Re[2,1]) < 0):
			beta = pi - p_beta
		else:
			beta = p_beta
	euler_zyx[0] = alpha
	euler_zyx[1] = beta
	euler_zyx[2] = gamma
	return euler_zyx

#齐次坐标系转换为六维坐标系
def T_to_Xzyx(T):
	'''
    :param T:齐次矩阵
    :return:
    '''
	#建立六维初始量
	X = np.zeros(6)
	#位置读取
	X[0:3] = T[0:3,3]
	#旋转矩阵变为欧拉角zyx
	X[3:6] = rot2euler_zyx(T[0:3,0:3])
	return X

#ZYX欧拉角速度变为姿态角速度转化矩阵
def J_euler_zyx(Phi):
	'''
		ZYX欧拉角速度变为姿态角速度转化矩阵
		input:欧拉角
		output:角速度变换矩阵
	'''
	J = np.array([[0, -np.sin(Phi[0]), np.cos(Phi[0])*np.cos(Phi[1])],
		[0, np.cos(Phi[0]), np.sin(Phi[0])*np.cos(Phi[1])],
		[1, 0, -np.sin(Phi[1])]])
	return J

#旋转矩阵到四元数
def rot_to_quaternion(R):
	'''
	:param R:
	:return:
	'''
	#四元数
	Q = np.zeros(4)
	if abs(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) > math.pow(10, -6):
		Q[0] = math.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2])/2.0
		Q[1] = 0.25 * (R[2, 1] - R[1, 2]) / Q[0]
		Q[2] = 0.25 * (R[0, 2] - R[2, 0]) / Q[0]
		Q[3] = 0.25 * (R[1, 0] - R[0, 1]) / Q[0]

	elif abs(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) > math.pow(10, -6):
		q = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])/2.0
		if (q*(R[1, 2] + R[2, 1]) > 0):
			Q[1] = q
		else:
			Q[1] = -q
		Q[0] = 0.25 * (R[1, 2] + R[2, 1]) / Q[1]
		Q[2] = 0.25 * (R[0, 1] + R[1, 0]) / Q[1]
		Q[3] = 0.25 * (R[0, 2] + R[2, 0]) / Q[1]

	elif abs(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) > math.pow(10, -6):
		q = math.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) / 2.0
		if (q * (R[0, 2] - R[2, 0]) > 0):
			Q[2] = q
		else:
			Q[2] = -q
		Q[0] = 0.25 * (R[0, 2] - R[2, 0]) / Q[2]
		Q[1] = 0.25 * (R[0, 1] + R[1, 0]) / Q[2]
		Q[3] = 0.25 * (R[1, 2] + R[2, 1]) / Q[2]
	else:
		q = math.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) / 2.0
		if (q * (R[1, 0] - R[0, 1]) > 0):
			Q[3] = q
		else:
			Q[3] = -q
		Q[0] = 0.25 * (R[1, 0] - R[1, 2]) / Q[3]
		Q[1] = 0.25 * (R[0, 2] + R[2, 0]) / Q[3]
		Q[2] = 0.25 * (R[1, 2] + R[2, 1]) / Q[3]
	return Q

#四元数到旋转矩阵
def quaternion_to_rot(Q):
	'''
	:param Q:
	:return:
	'''
	#建立旋转矩阵
	R = np.zeros([3, 3])

	#求取对应值
	R[0, 0] = Q[0] ** 2 + Q[1] ** 2 - Q[2] ** 2 - Q[3] ** 2
	R[1, 1] = Q[0] ** 2 - Q[1] ** 2 + Q[2] ** 2 - Q[3] ** 2
	R[2, 2] = Q[0] ** 2 - Q[1] ** 2 - Q[2] ** 2 + Q[3] ** 2

	R[0, 1] = 2 * (Q[1] * Q[2] - Q[3] * Q[0])
	R[0, 2] = 2 * (Q[1] * Q[3] + Q[2] * Q[0])
	R[1, 0] = 2 * (Q[1] * Q[2] + Q[3] * Q[0])
	R[1, 2] = 2 * (Q[2] * Q[3] - Q[1] * Q[0])
	R[2, 0] = 2 * (Q[1] * Q[3] - Q[2] * Q[0])
	R[2, 1] = 2 * (Q[2] * Q[3] - Q[1] * Q[0])
	return R


#==================相邻关节齐次变换矩阵================#
def  trans(theta,alpha,a,d):
	'''
		本函数用于求取n自由度机械臂正运动学
		输入参数为DH参数，角度单位为rad，长度单位为mm
		参数分别为theta,alpha,a,d，为0维常数值
		返回齐次传递函数矩阵
	'''
	T = np.array([[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha),  a*math.cos(theta)],
		[math.sin(theta), math.cos(theta)*math.cos(alpha),  -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
		[0,               math.sin(alpha),                  math.cos(alpha),                  d                ],
		[0,               0,                                0,                                  1              ]])
	return T

#===================规划相关基础函数==================#
#***5次多项式插值***#
#5次多项式插值待定参数求取
def interp5_param(q0,qv0,qa0,qf,qvf,qaf,tf):
	'''
		本函数用于求取5次多项式插值的参数a
		input:q0,qv0,qa0起点位置速度加速度
			  qf,qvf,qaf终点位置速度加速度
			  tf终点时间
		output:A 6*n,对应5次插值参数
	'''
	n = len(q0)
	A = np.zeros([6,n])
	for i in range(n):
		A[0,i] = q0[i]
		A[1,i] = qv0[i]
		A[2,i] = qa0[i]/2.0
		A[3,i] = (20*(qf[i] - q0[i]) - (8*qvf[i] + 12*qv0[i])*tf + (qaf[i] - 3*qa0[i])*tf*tf)/(2*pow(tf,3))
		A[4,i] = (-30*(qf[i] - q0[i]) + (14*qvf[i] + 16*qv0[i])*tf - (2*qaf[i] - 3*qa0[i])*tf*tf)/(2*pow(tf,4))
		A[5,i] = (12*(qf[i] - q0[i]) - (60*qvf[i] + 6*qv0[i])*tf + (qaf[i] - qa0[i])*tf*tf)/(2*pow(tf,5))
	return A

#5次多项式插值求取给定时刻关节值
def interp5_data(A,t):
	'''
		本函数用于求取5次插值多项式对应关节的位置速度加速度
		input：A=[a0,a1,a2,a3,a4,a5],5次多项式系数
			   t求取点对应时刻
		output:Q = [qq,qv,qa] 对应时刻位置速度加速度
	'''
	a0 = A[0,:]; a1 = A[1,:]; a2 = A[2,:]; a3 = A[3,:]; a4 = A[4,:]; a5 = A[5,:]
	
	n = len(a0)
	qq = np.zeros(n)
	qv = np.zeros(n)
	qa = np.zeros(n)
	for i in range(n):
		qq[i] = a0[i] + a1[i]*t + a2[i]*pow(t,2) + a3[i]*pow(t,3) + a4[i]*pow(t,4) + a5[i]*pow(t,5)
		qv[i] = a1[i] + 2*a2[i]*t + 3*a3[i]*pow(t,2) + 4*a4[i]*pow(t,3) + 5*a5[i]*pow(t,4)
		qa[i] = 2*a2[i] + 6*a3[i]*t + 12*a4[i]*pow(t,2) + 20*a5[i]*pow(t,3)
	return [qq,qv,qa]

#5次多项式插值求取求取一段时间的离散数值点,多变量
def interp5rdPoly(q0,qv0,qa0,qf,qvf,qaf,tf,dt):
	'''
		本函数用5项插值将离散数据连续化，多变量
		input:q0,qv0,qa0起点位置速度加速度
			  qf,qvf,qaf终点位置速度加速度
			  tf终点时间，dt时间间隔
		output:[q,qv,qa] 对应时刻位置速度加速度
	'''
	n = len(q0)
	k = np.floor(tf/dt).astype(int) + 1
	
	#求取5次多项式插值系数
	a0 = np.zeros(n)
	a1 = np.zeros(n)
	a2 = np.zeros(n)
	a3 = np.zeros(n)
	a4 = np.zeros(n)
	a5 = np.zeros(n)
	for i in range(n):
		a0[i] = q0[i]
		a1[i] = qv0[i]
		a2[i] = qa0[i]/2.0
		a3[i] = (20*(qf[i] - q0[i]) - (8*qvf[i] + 12*qv0[i])*tf + (qaf[i] - 3*qa0[i])*tf*tf)/(2*pow(tf,3))
		a4[i] = (-30*(qf[i] - q0[i]) + (14*qvf[i] + 16*qv0[i])*tf - (2*qaf[i] - 3*qa0[i])*tf*tf)/(2*pow(tf,4))
		a5[i] = (12*(qf[i] - q0[i]) - (60*qvf[i] + 6*qv0[i])*tf + (qaf[i] - qa0[i])*tf*tf)/(2*pow(tf,5))
	
	#数据点求取
	qq = np.zeros([k,n])
	qv = np.zeros([k,n])
	qa = np.zeros([k,n])
	t_seq = np.linspace(0,tf,k)
	for i in range(k):
		t = t_seq[i]
		for j in range(n):
			qq[i,j] = a0[j] + a1[j]*t + a2[j]*pow(t,2) + a3[j]*pow(t,3) + a4[j]*pow(t,4) + a5[j]*pow(t,5)
			qv[i,j] = a1[j] + 2*a2[j]*t + 3*a3[j]*pow(t,2) + 4*a4[j]*pow(t,3) + 5*a5[j]*pow(t,4)
			qa[i,j] = 2*a2[j] + 6*a3[j]*t + 12*a4[j]*pow(t,2) + 20*a5[j]*pow(t,3)
	return [qq,qv,qa]

#5次多项式插值求取求取一段时间的离散数值点,单变量
def interp5rdPoly1(q0,qv0,qa0,qf,qvf,qaf,tf,dt):
	'''
		本函数用5项插值将离散数据连续化,单变量
		input:q0,qv0,qa0起点位置速度加速度
			  qf,qvf,qaf终点位置速度加速度
			  tf终点时间，dt时间间隔
		output:[qq,qv,qa] 对应时刻位置速度加速度
	'''
	k = np.floor(tf/dt).astype(int) + 1
	#求取5次多项式插值系数

	a0 = q0
	a1 = qv0
	a2 = qa0/2.0
	a3 = (20*(qf - q0) - (8*qvf + 12*qv0)*tf + (qaf - 3*qa0)*tf*tf)/(2*pow(tf,3))
	a4 = (-30*(qf - q0) + (14*qvf + 16*qv0)*tf - (2*qaf - 3*qa0)*tf*tf)/(2*pow(tf,4))
	a5 = (12*(qf - q0) - (6*qvf + 6*qv0)*tf + (qaf - qa0)*tf*tf)/(2*pow(tf,5))
	
	#数据点求取
	qq = np.zeros(k)
	qv = np.zeros(k)
	qa = np.zeros(k)
	tt = np.linspace(0,tf,k)
	for i in range(k):
		t = tt[i]
		qq[i] = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5)
		qv[i] = a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4)
		qa[i] = 2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3)
	return [qq,qv,qa]

#***三次样条曲线***#
#三弯构造法求取三次样条曲线参数M
def spline_param(t,f,v0,vn):
	'''
		本函数用于求取三次样条曲线参数M
		input:t,f,时间系列、数值点系列
			v0,vn，起点、结束点速度
		output:m,h对应点的加速度,时间间隔系列
	'''
	n = len(t)
	#求取时间间隔及平均值
	h = np.zeros(n-1)  #时间间隔
	c = np.zeros(n-1)  #平均速度
	for i in range(n-1):
		h[i] = t[i+1] - t[i]
		c[i] = (f[i+1] - f[i])/h[i]
	
	#平均加速度求取
	d = np.zeros(n)
	d[0] = 6.0*(c[0] - v0)/h[0]
	a = np.zeros(n-2)
	b = np.zeros(n-2)
	for i in range(n-2):
		a[i] = h[i]/(h[i] + h[i+1])  #对应λ
		b[i] = 1-a[i]                #对应
		d[i+1] = 6*(c[i+1] - c[i])/(h[i+1] + h[i])
	d[n-1] = 6.0*(vn - c[n-2])/h[n-2]
	
	#时间矩阵计算
	P = 2*np.eye(n)
	P[0,1] = 1
	P[n-1,n-2] = 1
	for i in range(n-2):
		P[i+1,i+2] = b[i]            #对应ab问题
		P[i+1,i] = a[i]
	#求取系数M
	m = np.dot(np.linalg.inv(P),d)
	#print h
	return [m,h]
	
#三弯构造法求取三次单变量样条曲线
def spline1(t,f,interval,v0,vn):
	'''
		三次样条曲线，n个采样点，第一边界条件的三弯距插值
		input:时间，采样值序列，插值间隔，初速度，末速度(单变量）
		output:[s,vel,acc] 返回插值后的位置，速度，加速度
	'''	
	n = len(t)
	#求取插样系数
	[m,h] = spline_param(t,f,v0,vn)
	
	#求取插值函数
	k = np.floor(t[n-1]/interval).astype(int) + 1
	s = np.zeros(k)
	vel = np.zeros(k)
	acc = np.zeros(k)
	tt = np.linspace(0,t[n-1],k)
	for i in range(k-1):
		for j in range(n-1):
			if (tt[i] >= t[j]) & (tt[i] < t[j+1]): #判断所属区间
				#位移
				s[i] = m[j]*math.pow(t[j+1] - tt[i],3)/(6*h[j]) +m[j+1]*math.pow( tt[i] - t[j],3)/(6*h[j]) + (f[j] - m[j]*math.pow(h[j],2)/6.0)*(t[j+1]-tt[i])/h[j]+ (f[j+1] - m[j+1]*math.pow(h[j],2)/6.0)*(tt[i]-t[j])/h[j]
				#速度
				vel[i] = -m[j]*math.pow(t[j+1] - tt[i],2)/(2*h[j]) +m[j+1]*math.pow( tt[i] - t[j],2)/(2*h[j]) + (f[j+1] - f[j])/h[j] - (m[j+1] - m[j])*h[j]/6
				#加速度
				acc[i] = m[j]*(t[j+1] - tt[i])/h[j] + m[j+1]*( tt[i] - t[j])/h[j]
	s[k-1] = f[n-1]
	vel[k-1] = vn
	acc[k-1] = m[n-1]
	
	return [s,vel,acc]

#三弯构造法求取三次样条曲线特定时刻的值
def spline_tt(t,f,current_time,m,h):
	'''
		三次样条曲线，n个采样点，第一边界条件的三弯距插值
		input:时间，采样值序列，当前时刻，采样点加速度，时间间隔系列
		output:[s,vel,acc] 单关键时刻系列点
	'''	
	n = len(t)
	tt = current_time
	#求取插值函数
	for j in range(n-1):
		if (tt >= t[j]) & (tt < t[j+1]): #判断所属区间
			#位移
			s = m[j]*math.pow(t[j+1] - tt,3)/(6*h[j]) + m[j+1]*math.pow( tt - t[j],3)/(6*h[j]) + (f[j] - m[j]*math.pow(h[j],2)/6.0)*(t[j+1]-tt)/h[j]+ (f[j+1] - m[j+1]*math.pow(h[j],2)/6.0)*(tt-t[j])/h[j]
			#速度
			vel = -m[j]*math.pow(t[j+1] - tt,2)/(2*h[j]) +m[j+1]*math.pow(tt - t[j],2)/(2*h[j]) + (f[j+1] - f[j])/h[j] - (m[j+1] - m[j])*h[j]/6
			#加速度
			acc = m[j]*(t[j+1] - tt)/h[j] + m[j+1]*(tt - t[j])/h[j]
			
		else:
			#位移
			s = f[n-1]
			#速度
			vel = 0
			#加速度
			acc = m[n-1]
	return [s,vel,acc]

#三弯构造法求取三次样条曲线特定时刻的值,有且仅有两个点
def spline_tt1(t,f,current_time,m,h):
	'''
		三次样条曲线，2个采样点，第一边界条件的三弯距插值
		input:时间，采样值序列，当前时刻，采样点加速度，时间间隔系列
		output:[s,vel,acc] 单关键时刻系列点
	'''	
	n = len(t)
	if n != 2:
		print "输入数据有误，本函数仅做两点间的插值!!!!!"
	tt = current_time
	#求取插值函数
	#位移
	s = m[0]*math.pow(t[1] - tt,3)/(6.0*h) + m[1]*math.pow( tt - t[0],3)/(6.0*h) + (f[0] - m[0]*math.pow(h,2)/6.0)*(t[1]-tt)/h + (f[1] - m[1]*math.pow(h,2)/6.0)*(tt-t[0])/h
	#速度
	vel = -m[0]*math.pow(t[1] - tt,2)/(2.0*h) +m[1]*math.pow(tt - t[0],2)/(2.0*h) + (f[1] - f[0])/(1.0*h) - (m[1] - m[0])*h/6.0
	#加速度??????
	acc = m[0]*(t[1] - tt)/h + m[1]*(tt - t[0])/h
	
	return [s,vel,acc]

#=================机械臂关节角极限及跳跃=================#
#机械臂关节极限判断，返回值为0或1
def exceed_joint_limit(qq,q_min,q_max):
	'''
		判断关节角是否超出限制
		input:关节角，关节角范围
		outpu：0,未超出，1超出
	'''
	n = len(qq)
	limit = 0
	for i in range(n):
		if((qq[i] < q_min[i]) or (qq[i] > q_max[i])):
			limit = 1
			break
	return limit

#判断关节是否有跳变
def joint_have_jump(qq,q_init,delt_max = 0.5):
	'''
		判断关节角是与上一时刻插值是否过大
		input:当前关节角，上一时刻关节角,最大差值
		outpu：0,没有，1有
	'''
	jump_label = 0
	n = len(qq)
	for i in range(n):
		if(np.abs(qq[i]-q_init[i]) > delt_max):
			jump_label = 1
			break
	return jump_label

#================关节角角度选择==================#
#建立U型函数
def U_function(qq,q_max,q_min):
	'''
	:param qq:
	:param q_max:
	:param q_min:
	:return:
	'''
	#建立系数
	c = 0.1
	n = 1

	#求取归一化偏差量
	x = (2*qq - (q_max + q_min))/(q_max - q_min)
	#对偏差进一步缩放

	xx = math.pow(x,2*n)
	#print "xx: %s" % xx
	#带入指数函数

	f = math.exp(c*xx) - 1
	#print "f: %s" % f

	return f

#解析解8选一算法
def qq_eight_choice(Q,qq_k):
	'''
	:param Q: 8组关节角，行排列
	:param qq_k: 上一时刻关节角
	:return:
	'''
	#定义当前关节角与上一时刻关节角的距离
	delta_l = np.zeros(8)
	#求取每组关节角与上一时刻关节角的距离
	for i in range(8):
		qq = Q[i, :]
		delta_qq = qq - qq_k
		delta_l[i] = nla.norm(delta_qq)
	l_min_k = np.argmin(delta_l)
	qq_choice =Q[l_min_k,:]
	return qq_choice

#将关节角计算到正负pi
def qq_choose(qq):
	'''
		本函数用于选着关节角范围
		input:qq为计算出的关节角
		output:q关节角范围[-pi,pi]
	'''
	n = len(qq)
	q = np.copy(qq)
	for i in range(n):
		while (q[i] > pi):
			q[i] = q[i] - 2*pi
		while (q[i] < - pi):
			q[i] = q[i] + 2*pi
	return q

#===================观测器=====================#
#***高增益观测器***#
#高增益观测器,采用双曲线性变化离散化，求取观测值的导数
def high_gain_observer(qq,qv,T,alpha1=10,alpha2=10):
	'''
	:param qq: 前三个时刻的采集值
	:param qv: 前两个时刻的评估值
	:param alpha1: 观测器参数
	:param alpha2:
	:param T: 采样周期
	:return:
	'''
	#中间参数
	b = 2*alpha2/T*(4+2*alpha1+alpha2)
	a1 = (2*alpha2 - 8)/(4 + 2*alpha1 + alpha2)
	a2 = (4 - 2*alpha1 + alpha2)/(4 + 2*alpha1 + alpha2)

	#求当前时刻的评估速度
	q_v = (b*qq[0] - b*qq[2] - qv[0] - a1*qv[1])/a2

	#更新速度系列
	qv_new = np.zeros(2)
	qv_new[0] = qv[1]
	qv_new[1] = q_v
	return qv_new

#高增益观测测器，观测多变量
def high_gain_observer_multi_var(qq,qv,T):
	'''
	:param qq:
	:param qv:
	:param T:
	:return:
	'''
	#观测变量个数
	n = len(qq[0,:])
	#评估的导数
	qv_new = np.zeros([2,n])
	for i in range(n):
		qv_new[:,i] = high_gain_observer(qq[:,i],qv[:,i],T)
	return qv_new






