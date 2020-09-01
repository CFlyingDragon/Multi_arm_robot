#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于求7自由度SRS型机械臂运动学相关函数
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.9.20
import numpy as np
import numpy.linalg as nla
import math
from math import pi

#导入自定义函数
import BaseFunction as bf
from RobotParameter import q_limit_armc
from RobotParameter import DH0_armc
import RobotParameter as rp

#UR5机械臂

#=================相邻关节齐次变换矩阵=================#
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

#=================已知关节角求取臂形角=================#
def arm_angle_by_joint(qq,DH_0):
	'''
		计算臂形角通过关节角
		input:当前关节角，DH参数
		outpu：臂型角
	'''
	theta = DH_0[:,0] + qq
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]

	Ai = trans(theta[0],alpha[0],a[0],d[0])
	S = Ai[0:3,3]
	for i in range(2):
		Ai = np.dot(Ai,trans(theta[i+1],alpha[i+1],a[i+1],d[i+1]))
	E = Ai[0:3,3]
	for i in range(2):
		Ai = np.dot(Ai,trans(theta[i+3],alpha[i+3],a[i+3],d[i+3]))
	W = Ai[0:3,3]

	V = np.array([0, 0, 1])
	w = W - S
	e = E - S

	ww = w/nla.norm(w)
	p = e - np.dot(ww,e)*ww

	psi = np.arctan2(np.dot(ww,np.cross(V,p)),np.dot(V,p))

	return psi

#================建立n自由度机械臂正运动学==============#
#输入某时刻对应的DH,返回齐次矩阵
def fkine(theta,alpha,a,d):
	'''
		本函数用于求取n自由度机械臂正运动学
		输入参数为DH参数，角度单位为rad，长度单位为mm
		参数分别为theta,alpha,a,d，为1维常数值
		返回齐次传递函数矩阵 
	'''
	#关节自由度
	n = len(theta)
	#建立4×4的齐次传递矩阵,定义为numpy类型
	An = np.eye(4)
	for i  in range(n):
		T =  trans(theta[i],alpha[i],a[i],d[i])
		An = np.dot(An,T)  #末端到惯性坐标系传递矩阵
	return An

#输入初始时刻DH_0和相对转角,输出六维末端位姿
def fkine_euler(DH_0,qr):
	'''
		本函数用于求取n自由度机械臂正运动学
		输入参数为DH参数，角度单位为rad，长度单位为mm
		参数分别为theta,alpha,a,d，为1维常数值
		返回齐次传递函数矩阵
	'''
	#DH参数
	theta = DH_0[:, 0] + qr
	alpha = DH_0[:, 1]
	a = DH_0[:, 2]
	d = DH_0[:, 3]
	#关节自由度
	n = len(theta)
	xe = np.zeros(6)
	#建立4×4的齐次传递矩阵,定义为numpy类型
	An = np.eye(4)
	for i  in range(n):
		T =  trans(theta[i],alpha[i],a[i],d[i])
		An = np.dot(An,T)  #末端到惯性坐标系传递矩阵
	xe[0:3] = An[0:3,3]
	xe[3:6] = bf.rot2euler_zyx(An[0:3,0:3])

	return xe

#=====================求雅克比矩阵====================#
#构造法求雅克比矩阵,时间0.3ms
def jacobian(DH_0,qr):
	'''
		本函数用于求取机械臂的雅克比矩阵
		input:DH_0参数，长度单位mm,角度单位red
			  qr,相对初始位置的转角
		output:J,该位置点的雅克比矩阵
	'''
	n = len(qr)
	theta = DH_0[:,0] + qr
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	
	#求取末端位置
	An = fkine(theta,alpha,a,d)
	p_n = An[0:3,3]
	J = np.zeros([6,n])
	J[3:6,n-1] = An[0:3,2]
	
	#求取其余转轴方向及位置点
	Ai = np.eye(4)
	for i in range(n-1):
		z_i = Ai[0:3,2]
		p_i = Ai[0:3,3]
		p_in = p_n - p_i
		J[0:3,i] = np.cross(z_i,p_in)
		J[3:6,i] = z_i
		Ai = np.dot(Ai, trans(theta[i], alpha[i], a[i], d[i]))
	return J

#迭代雅可比,运行时间更快0.1ms
def jeco_0(DH_0, qr):
	'''
		本函数基于雅克比迭代求解n自由度机械臂逆运动学方程
		input:DH_0 = [q_init,alpha,a,d];
			 q_ready是上一时刻的位置,单位:弧度;
		     T0e为DH坐标系确定的DH{0}坐标系与DH{6}之间的关系(目标矩阵);
		     efs求解误差阀值，默认值10^(-10)
			 i_limit迭代最大次数,默认值1000			  
		output:qq为相对与DH_q0的转动角度,单位:弧度;已处理到[-pi, pi] 之间
	'''
	#建立初时刻迭代初值
	q = DH_0[:,0] + qr
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	
	#计数及标签
	n = len(q)
						  
	#计算雅克比矩阵
	U = np.eye(4)
	Jn = np.zeros([6,n])
	T = np.zeros([4,4,n])
	for i in range(n):
		i = n - i - 1
		T[:,:,i] = trans(q[i],alpha[i],a[i],d[i])
		U = np.dot(T[:,:,i],U)
		dd = np.array([-U[0,0]*U[1,3] + U[1,0]*U[0,3],
						-U[0,1]*U[1,3] + U[1,1]*U[0,3],
						-U[0,2]*U[1,3] + U[1,2]*U[0,3]])
		Jn[0:3,i] = dd
		Jn[3:6,i] = U[2,0:3] 
	
	An = fkine(q,alpha,a,d)		
	R = An[0:3,0:3]
	J_R = np.zeros([6,6])
	J_R[0:3,0:3] = R
	J_R[3:6,3:6] = R
		
	J0 = np.dot(J_R,Jn)
	return J0

#==============采用数值解求解机械臂逆运动学==============#
#***基于雅克比矩阵迭代求解逆运动学***#
def iterate_ikine(DH_0, q_ready, T0e, efs = pow(10,-12), i_max = 1000):
	'''
		本函数基于雅克比迭代求解n自由度机械臂逆运动学方程
		input:DH_0 = [q_init,alpha,a,d];
			 q_ready是上一时刻的位置,单位:弧度;
		     T0e为DH坐标系确定的DH{0}坐标系与DH{6}之间的关系(目标矩阵);
		     efs求解误差阀值，默认值10^(-10)
			 i_limit迭代最大次数,默认值1000			  
		output:qq为相对与DH_q0的转动角度,单位:弧度;已处理到[-pi, pi] 之间
	'''
	#建立初时刻迭代初值
	q_r = DH_0[:,0] + q_ready
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	
	#计数及标签
	n = len(q_r)
	deltaQ = 1  
	temp_count = 0
	
	#迭代循环求解
	while (deltaQ > efs):
		
		#求解正运动学
		An = np.eye(4)
		T = np.zeros([4,4,n])
		
		for i in range(n):
			T[:,:,i] = trans(q_r[i],alpha[i],a[i],d[i])
			An = np.dot(An,T[:,:,i])
			
		#计算末端误差
		dA = np.zeros(6)
		dA[0:3] = T0e[0:3,3] - An[0:3,3]
		dA[3:6] = 0.5*(np.cross(An[0:3,0],T0e[0:3,0]) + np.cross(An[0:3,1],T0e[0:3,1]) 
				+ np.cross(An[0:3,2],T0e[0:3,2]))
	
		#print dA						  
		#计算雅克比矩阵
		U = np.eye(4)
		Jn = np.zeros([6,n])
		for i in range(n):
			i = n - i - 1
			U = np.dot(T[:,:,i],U)
			
			dd = np.array([ -U[0,0]*U[1,3] + U[1,0]*U[0,3],
							-U[0,1]*U[1,3] + U[1,1]*U[0,3],
							-U[0,2]*U[1,3] + U[1,2]*U[0,3]])
			Jn[0:3,i] = dd
			Jn[3:6,i] = U[2,0:3]
			
		R = An[0:3,0:3]
		J_R = np.zeros([6,6])
		J_R[0:3,0:3] = R
		J_R[3:6,3:6] = R
		
		J0 = np.dot(J_R,Jn)
		#求取关节角关节角度偏差值
		dq = np.dot(np.linalg.pinv(J0),dA)
		q_r = q_r + dq
		deltaQ = np.linalg.norm(dq)
		temp_count =temp_count + 1
		if (temp_count > i_max):
			print("Solution wouldn't converge")
			return q_ready

	q_tmp = q_r - DH_0[:,0]		
	q = bf.qq_choose(q_tmp)
	return q

#***基于梯度投影迭代求解逆运动学***#
#梯度项
def jaco_grad(q,J,J0_pinv,q_min = q_limit_armc[0,:],q_max= q_limit_armc[1,:],k = 0.1):
	'''
	:param q:
	:param J:
	:param J0_pinv:
	:param q_min:
	:param q_max:
	:param k:
	:return:
	'''
	J_N = np.eye(7) - np.dot(J0_pinv,J)
	delta_H = np.zeros(7)
	for i in range(7):
		ai = 1/2.0*(q_min[i] + q_max[i])
		delta_H[i] = 2*(q[i] - ai)/math.pow(ai - q_max[i],2)
	qv_N = k*np.dot(J_N,delta_H)

	return qv_N

#基于梯度投影迭代求解逆运动学
def grad_iterate_ikine(DH_0, q_ready, T0e, efs=pow(10, -12), i_max=1000):
	'''
		本函数基于雅克比迭代求解n自由度机械臂逆运动学方程
		input:DH_0 = [q_init,alpha,a,d];
			 q_ready是上一时刻的位置,单位:弧度;
		     T0e为DH坐标系确定的DH{0}坐标系与DH{6}之间的关系(目标矩阵);
		     efs求解误差阀值，默认值10^(-10)
			 i_limit迭代最大次数,默认值1000
		output:qq为相对与DH_q0的转动角度,单位:弧度;已处理到[-pi, pi] 之间
	'''
	# 建立初时刻迭代初值
	q_r = DH_0[:, 0] + q_ready
	alpha = DH_0[:, 1]
	a = DH_0[:, 2]
	d = DH_0[:, 3]

	# 计数及标签
	LimitFlag = 0
	n = len(q_r)
	deltaQ = 1
	temp_count = 0
	E = np.eye(4)
	# 迭代循环求解
	while (deltaQ > efs) & (LimitFlag == 0):

		# 求解正运动学
		An = np.eye(4)
		T = np.zeros([4, 4, n])

		for i in range(n):
			T[:, :, i] = trans(q_r[i], alpha[i], a[i], d[i])
			An = np.dot(An, T[:, :, i])

		# 计算末端误差
		dA = np.zeros(6)
		dA[0:3] = T0e[0:3, 3] - An[0:3, 3]
		dA[3:6] = 0.5 * (np.cross(An[0:3, 0], T0e[0:3, 0]) + np.cross(An[0:3, 1], T0e[0:3, 1])
						 + np.cross(An[0:3, 2], T0e[0:3, 2]))

		# print dA
		# 计算雅克比矩阵
		U = np.eye(4)
		Jn = np.zeros([6, n])
		for i in range(n):
			i = n - i - 1
			U = np.dot(T[:, :, i], U)

			dd = np.array([-U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3],
						   -U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3],
						   -U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3]])
			Jn[0:3, i] = dd
			Jn[3:6, i] = U[2, 0:3]

		R = An[0:3, 0:3]
		J_R = np.zeros([6, 6])
		J_R[0:3, 0:3] = R
		J_R[3:6, 3:6] = R

		J0 = np.dot(J_R, Jn)
		J0_pinv = np.linalg.pinv(J0)
		# 求取关节角关节角度偏差值
		dq = np.dot(J0_pinv, dA) + jaco_grad(q_r,J0,J0_pinv,k=0)
		q_r = q_r + dq
		deltaQ = np.linalg.norm(dq)
		temp_count = temp_count + 1
		if (temp_count > i_max):
			q_tmp = q_r - DH_0[:, 0]
			q = bf.qq_choose(q_tmp)
			LimitFlag = 1
			print("Solution wouldn't converge")
	q_tmp = q_r - DH_0[:, 0]
	q = bf.qq_choose(q_tmp)
	return q

#***加权最小范数法求取冗余机械臂的速度/位置逆运动学***#
#加权最小范数求机器人的速度级逆
def Weighted_minimum_norm_velocity(DH_0,qr,q_max,q_min,d_h0,X_dot):
	'''
	:param DH_0: DH参数
	:param qr: 参考关节角
	:param q_max: 最大关节角
	:param q_min: 最小关节角
	:param d_h0: 上一时刻优化函数偏导
	:param X_dot: 末端六维速度
	:return: q_v,关节速度；d_h,优化函数偏导
	'''
	#关节个数
	n = len(qr)
	#求取雅克比矩阵
	J = jeco_0(DH_0, qr)
	#求取加权系数
	W = np.zeros([n,n])
	d_h = np.zeros(n) #优化函数对关节角的偏导数
	for i in range(n):
		d_h[i] = 0.25*math.pow(q_max[i]-q_min[i],2)*(2*qr[i]-q_max[i]-q_min[i])/\
			  math.pow((q_max[i]-qr[i])*(qr[i]-q_min[i]),2)
		dd_h =d_h[i] - d_h0[i]
		if(dd_h>0 or dd_h==0):
			W[i,i] = 1 + d_h[i]
		else:
			W[i,i] = 1
	#求取关节速度
	Jw = np.dot(nla.inv(W),J.T)
	Jw_inv = np.dot(Jw,nla.inv(np.dot(J,Jw)))
	#Jw_inv = nla.pinv(J)
	q_v = np.dot(Jw_inv,X_dot)
	return [q_v,d_h]

#加权最小范数求机器人的位置级逆,时间45ms
def w_ikine(DH_0,qr,q_max,q_min,d_h0,T0e,efs=pow(10, -12), i_max=1000):
	'''
	:param DH_0:
	:param qr:
	:param q_max:
	:param q_min:
	:param d_h0:
	:param X:
	:return:
	'''
	# 建立初时刻迭代初值
	q_r = DH_0[:, 0] + qr
	alpha = DH_0[:, 1]
	a = DH_0[:, 2]
	d = DH_0[:, 3]

	# 计数及标签
	n = len(q_r)
	deltaQ = 1
	temp_count = 0
	dh_temp = d_h0
	dh = np.zeros(n)

	# 迭代循环求解
	while (deltaQ > efs):

		# 求解正运动学
		An = fkine(q_r,alpha,a,d)

		# 计算末端误差
		dA = np.zeros(6)
		dA[0:3] = T0e[0:3, 3] - An[0:3, 3]
		dA[3:6] = 0.5 * (np.cross(An[0:3, 0], T0e[0:3, 0]) + np.cross(An[0:3, 1], T0e[0:3, 1])
						 + np.cross(An[0:3, 2], T0e[0:3, 2]))

		# 求取关节角度偏差值
		[dq,dh] = Weighted_minimum_norm_velocity(DH_0,qr,q_max,q_min,dh_temp,dA)
		q_r = q_r + dq
		deltaQ = np.linalg.norm(dq)
		temp_count = temp_count + 1
		if (temp_count > i_max):
			print("Solution wouldn't converge")
			return [qr,dh]
		dh_temp = dh

	q_tmp = q_r - DH_0[:, 0]
	qq = bf.qq_choose(q_tmp)
	return [qq,dh]

# ============采用解析解求取SRS无偏置型机器人逆运动学==============#
#***基于双臂型角求解析解,DH参数特定（初始角为0),运行时间0.45ms***#
#采用z0与sw做参考面，求取第一臂形角8组解,求取运行时间0.3ms
def arm_angle_ikine_first(DH,psi,Te):
	'''
	:param DH:
	:param psi:
	:param Te:
	:return:
	'''
	#DH参数
	d1 = DH[0, 3]
	d3 = DH[2, 3]
	d5 = DH[4, 3]
	d7 = DH[6, 3]
	#求取SW
	s = np.array([0,0,d1])
	z0 = np.array([0,0,1])
	w = Te[0:3,3] - d7*Te[0:3,2]
	sw = w - s
	l_sw = nla.norm(sw)
	u_sw = sw/l_sw
	#***求取关节角4,第二个参数分别代表q4两组解***#
	#计算beta值
	beta = np.arccos((d3**2 + d5**2 - l_sw**2)/(2*d3*d5))
	#关节角
	qq4_1 = pi - beta #上
	qq4_2 = beta - pi #下
	qq4 = np.array([qq4_1,qq4_2])

	# ***求取关节角1,2,3,第三个参数分别代表q2两组解***#
	#求取臂型角到到旋转矩阵
	R_psi = bf.euler_axis2rot(u_sw,psi)
	#求取夹角alpha
	alpha_ = np.arccos((d3**2 + l_sw**2 - d5**2)/(2*d3*l_sw))
	#定义转轴,用于求y_3_0
	V = z0 #参考轴,第一参考轴
	l = np.cross(sw,V)  #参考平面法向量
	u_l = l/nla.norm(l)

	#对应上下臂形
	alpha = np.array([alpha_, - alpha_])

	#定义8组关节角
	Q = np.zeros([8,7])

	#i=0代表上臂形,i=1代表下臂形
	for i in range(2):
		#求取y3_0
		y3_0 = -np.dot(bf.euler_axis2rot(u_l, alpha[i]), u_sw)
		#求取x3_0
		x3_0 = (sw + (d3 + d5 * np.cos(qq4[i])) * y3_0) / (d5 * np.sin(qq4[i]))
		#求取z3_0
		z3_0 = np.cross(x3_0,y3_0)
		z3_0 = z3_0/nla.norm(z3_0)

		#求取坐标系3在初始位置下在基座标系中的表示
		R_03_0 = np.zeros([3,3])
		R_03_0[:, 0] = x3_0
		R_03_0[:, 1] = y3_0
		R_03_0[:, 2] = z3_0
		#求取3号坐标系在任意位置在基坐标系中的表示
		R_03 = np.dot(R_psi,R_03_0)

		#求取关节角2,有两组解
		qq2_1 = np.arctan2(np.sqrt(1 - R_03[2, 1] ** 2), -R_03[2,1])
		qq2_2 = np.arctan2(-np.sqrt(1 - R_03[2, 1] ** 2), -R_03[2, 1])

		#求取关节角1
		qq1_1 = np.arctan2(-R_03[1, 1] * np.sin(qq2_1), -R_03[0, 1] * np.sin(qq2_1))
		qq1_2 = np.arctan2(-R_03[1, 1] * np.sin(qq2_2), -R_03[0, 1] * np.sin(qq2_2))

		#求取关节角3
		qq3_1 = np.arctan2(R_03[2, 2] * np.sin(qq2_1), -R_03[2, 0] * np.sin(qq2_1))
		qq3_2 = np.arctan2(R_03[2, 2] * np.sin(qq2_2), -R_03[2, 0] * np.sin(qq2_2))

		#****求取关节角5,6,7****#
		#求变换矩阵,第二个下标是参考坐标系
		R4_3 = np.array([[np.cos(qq4[i]), 0, np.sin(qq4[i])],
						 [np.sin(qq4[i]), 0, -np.cos(qq4[i])],
						 [0				, 1, 0]])
		R4 = np.dot(R_psi,np.dot(R_03_0,R4_3))
		R4_7 = np.dot(Te[0:3,0:3].T,R4)
		R_47 = R4_7.T

		#求取关节角6,存在两组解
		qq6_1 = np.arctan2(np.sqrt(1 - R_47[2, 2] ** 2), R_47[2, 2])
		qq6_2 = np.arctan2(-np.sqrt(1 - R_47[2, 2] ** 2), R_47[2, 2])

		# 求取关节角1
		qq5_1 = np.arctan2(R_47[1, 2] * np.sin(qq6_1), R_47[0, 2] * np.sin(qq6_1))
		qq5_2 = np.arctan2(R_47[1, 2] * np.sin(qq6_2), R_47[0, 2] * np.sin(qq6_2))

		# 求取关节角3
		qq7_1 = np.arctan2(R_47[2, 1] * np.sin(qq6_1), -R_47[2, 0] * np.sin(qq6_1))
		qq7_2 = np.arctan2(R_47[2, 1] * np.sin(qq6_2), -R_47[2, 0] * np.sin(qq6_2))

		#求解8组解
		q = np.array([[qq1_1, qq2_1, qq3_1, qq4[i], qq5_1, qq6_1, qq7_1],
					  [qq1_1, qq2_1, qq3_1, qq4[i], qq5_2, qq6_2, qq7_2],
					  [qq1_2, qq2_2, qq3_2, qq4[i], qq5_1, qq6_1, qq7_1],
					  [qq1_2, qq2_2, qq3_2, qq4[i], qq5_2, qq6_2, qq7_2]])
		if(i==0):
			Q[0:4,:] = q
		else:
			Q[4:8,:] = q
	return Q

#采用x0与sw做参考面，求取第二臂形角8组解，时间0.3ms
def arm_angle_ikine_second(DH,psi,Te):
	'''
	:param DH:
	:param psi:
	:param Te:
	:return:
	'''
	#DH参数
	d1 = DH[0, 3]
	d3 = DH[2, 3]
	d5 = DH[4, 3]
	d7 = DH[6, 3]
	#求取SW
	s = np.array([0,0,d1])
	x0 = np.array([1,0,0])
	w = Te[0:3,3] - d7*Te[0:3,2]
	sw = w - s
	l_sw = nla.norm(sw)
	u_sw = sw/l_sw
	#***求取关节角4,第二个参数分别代表q4两组解***#
	#计算beta值
	beta = np.arccos((d3**2 + d5**2 - l_sw**2)/(2*d3*d5))
	#关节角
	qq4_1 = pi - beta #上
	qq4_2 = beta - pi #下
	qq4 = np.array([qq4_1,qq4_2])

	# ***求取关节角1,2,3,第三个参数分别代表q2两组解***#
	#求取臂型角到到旋转矩阵
	R_psi = bf.euler_axis2rot(u_sw,psi)
	#求取夹角alpha
	alpha_ = np.arccos((d3**2 + l_sw**2 - d5**2)/(2*d3*l_sw))
	#定义转轴,用于求y_3_0
	V = x0 #定义参考轴,第二参考轴
	l = np.cross(sw,V) #参考平面法向量
	u_l = l/nla.norm(l)

	#对应上下臂形
	alpha = np.array([alpha_, - alpha_])

	#定义8组关节角
	Q = np.zeros([8,7])

	#i=0代表上臂形,i=1代表下臂形
	for i in range(2):
		#求取y3_0
		y3_0 = -np.dot(bf.euler_axis2rot(u_l, alpha[i]), u_sw)
		#求取x3_0
		x3_0 = (sw + (d3 + d5 * np.cos(qq4[i])) * y3_0) / (d5 * np.sin(qq4[i]))
		#求取z3_0
		z3_0 = np.cross(x3_0,y3_0)
		z3_0 = z3_0/nla.norm(z3_0)

		#求取坐标系3在初始位置下在基座标系中的表示
		R_03_0 = np.zeros([3,3])
		R_03_0[:, 0] = x3_0
		R_03_0[:, 1] = y3_0
		R_03_0[:, 2] = z3_0
		#求取3号坐标系在任意位置在基坐标系中的表示
		R_03 = np.dot(R_psi,R_03_0)

		#求取关节角2,有两组解
		qq2_1 = np.arctan2(np.sqrt(1 - R_03[2, 1] ** 2), -R_03[2,1])
		qq2_2 = np.arctan2(-np.sqrt(1 - R_03[2, 1] ** 2), -R_03[2, 1])

		#求取关节角1
		qq1_1 = np.arctan2(-R_03[1, 1] * np.sin(qq2_1), -R_03[0, 1] * np.sin(qq2_1))
		qq1_2 = np.arctan2(-R_03[1, 1] * np.sin(qq2_2), -R_03[0, 1] * np.sin(qq2_2))

		#求取关节角3
		qq3_1 = np.arctan2(R_03[2, 2] * np.sin(qq2_1), -R_03[2, 0] * np.sin(qq2_1))
		qq3_2 = np.arctan2(R_03[2, 2] * np.sin(qq2_2), -R_03[2, 0] * np.sin(qq2_2))

		#****求取关节角5,6,7****#
		#求变换矩阵,第二个下标是参考坐标系
		R4_3 = np.array([[np.cos(qq4[i]), 0, np.sin(qq4[i])],
						 [np.sin(qq4[i]), 0, -np.cos(qq4[i])],
						 [0				, 1, 0]])
		R4 = np.dot(R_psi,np.dot(R_03_0,R4_3))
		R4_7 = np.dot(Te[0:3,0:3].T,R4)
		R_47 = R4_7.T

		#求取关节角6,存在两组解
		qq6_1 = np.arctan2(np.sqrt(1 - R_47[2, 2] ** 2), R_47[2, 2])
		qq6_2 = np.arctan2(-np.sqrt(1 - R_47[2, 2] ** 2), R_47[2, 2])

		# 求取关节角1
		qq5_1 = np.arctan2(R_47[1, 2] * np.sin(qq6_1), R_47[0, 2] * np.sin(qq6_1))
		qq5_2 = np.arctan2(R_47[1, 2] * np.sin(qq6_2), R_47[0, 2] * np.sin(qq6_2))

		# 求取关节角3
		qq7_1 = np.arctan2(R_47[2, 1] * np.sin(qq6_1), -R_47[2, 0] * np.sin(qq6_1))
		qq7_2 = np.arctan2(R_47[2, 1] * np.sin(qq6_2), -R_47[2, 0] * np.sin(qq6_2))

		#求解8组解
		q = np.array([[qq1_1, qq2_1, qq3_1, qq4[i], qq5_1, qq6_1, qq7_1],
					  [qq1_1, qq2_1, qq3_1, qq4[i], qq5_2, qq6_2, qq7_2],
					  [qq1_2, qq2_2, qq3_2, qq4[i], qq5_1, qq6_1, qq7_1],
					  [qq1_2, qq2_2, qq3_2, qq4[i], qq5_2, qq6_2, qq7_2]])
		if(i==0):
			Q[0:4,:] = q
		else:
			Q[4:8,:] = q
	return Q

#基于臂型角逆解运算,选择最佳关节角,并判断是否超出关节极限
def arm_angle_ikine_chose(Q, qk, q_min,q_max):
	'''
	:param Q:8组解析解
	:param qk:上一时刻的解
	:param q_max:
	:param q_min:
	:return:返回最佳关节角和极限标签
	'''
	#是否超关节极限
	limit_flag = False
	#求取Q的形状
	m = len(Q[:,0])
	#创建中间变量
	dq = np.zeros(m)
	#判断关节角是否超出极限//后期做处理
	for i in range(m):
		if (bf.exceed_joint_limit(Q[i,:],q_min,q_max)):
			dq[i] = 1000.0 + i #超出关节极限
		else:
			#关节角未超出极限，求取与上一时刻偏差
			dqqi = Q[i,:] - qk
			dq[i] = nla.norm(dqqi)
	#求取与上一时刻偏差最小的一组
	dq_min = min(dq)
	#判断是否有解
	if(dq_min >= 1000.0):
		print "关节角超出关节极限：无解"
		limit_flag = True
	#选取最佳关节角
	dq_minIndex = np.where(dq == dq_min)
	qq =np.array(Q[dq_minIndex,:]).reshape(1,-1)[0] #转换成向量
	return [qq,limit_flag]

#选择求取关节所需参考面,并调用对应算法求取8组关节角
def arm_angle_all_ikine( DH_0, psi,Te):
	'''
	:param DH_0:
	:param psi:
	:param Te:
	:return:
	'''
	##判断臂形角是否奇异
	d1 = DH_0[0, 3]
	d7 = DH_0[6, 3]
	s = np.array([0, 0, d1])
	w = Te[0:3, 3] - d7 * Te[0:3, 2]
	sw = w - s
	if (np.abs(sw[0] < pow(10, -6)) and np.abs(sw[1] < pow(10, -6))):
		print "臂形角奇异,采用第二臂形角求解"
		#计算第二臂型角
		phi = math.atan2(-sw[0]*sw[1],-sw[0]**2) #与是否为单位向量无关
		psi_x = phi + psi
		#调用第二臂形角计算
		Q = arm_angle_ikine_second(DH_0, psi_x,Te)
	else:
		Q = arm_angle_ikine_first(DH_0, psi, Te)
	return Q

#基于臂型角逆解运算,单一值输出接口,运行时间0.4ms
def arm_angle_ikine(Te, psi, qk, DH_0, qq_min, qq_max):
	'''
	:param Te:末端矩阵
	:param psi:第一臂形角
	:param qk:上一时刻关节角
	:param DH_0:
	:param q_min:
	:param qq_max:
	:return:关节角和求解标签
	'''
	#建立求解是否成功标签
	succeed_label = True  # 默认成功求逆

	#调用求解函数求取8组解
	Q = arm_angle_all_ikine(DH_0, psi, Te)

	[q,limit_flag] = arm_angle_ikine_chose(Q, qk, qq_min,qq_max)
	if(limit_flag):
		print "求解失败！"
		succeed_label = False

	return [q, succeed_label]

# ============采用解析解求取SRS无偏置型机器人避关节极限逆运动学==============#
#***基于双臂型角求解析解,DH参数特定（初始角为0),运行时间0.6ms***#
#采用z0与sw做参考面，求取第一臂形角8组解,求取运行时间0.3ms
def arm_angle_ikine_first_limit(DH,Te):
	'''
	:param DH:
	:param Te:
	:return:
	'''
	#DH参数
	d1 = DH[0, 3]
	d3 = DH[2, 3]
	d5 = DH[4, 3]
	d7 = DH[6, 3]
	#求取SW
	s = np.array([0,0,d1])
	z0 = np.array([0,0,1])
	w = Te[0:3,3] - d7*Te[0:3,2]
	sw = w - s
	l_sw = nla.norm(sw)
	u_sw = sw/l_sw
	#***求取关节角4,第二个参数分别代表q4两组解***#
	#计算beta值
	beta = np.arccos((d3**2 + d5**2 - l_sw**2)/(2*d3*d5))
	#关节角
	qq4_1 = pi - beta #上
	qq4_2 = beta - pi #下
	qq4 = np.array([qq4_1,qq4_2])

	#求解关节中心点旋转矩阵,由于关节极限对称,所以为0
	R_03_mid = np.array([[1, 0, 0],
						 [0, 0, 1],
						 [0, -1, 0]])

	R_74_mid = np.array([[1, 0, 0],
						 [0, 1, 0],
						 [0, 0, 1]])

	# ***求取关节角1,2,3,第三个参数分别代表q2两组解***#
	#求取夹角alpha
	alpha_ = np.arccos((d3**2 + l_sw**2 - d5**2)/(2*d3*l_sw))
	#定义转轴,用于求y_3_0
	V = z0 #参考轴,第一参考轴
	l = np.cross(sw,V)  #参考平面法向量
	u_l = l/nla.norm(l)

	#对应上下臂形
	alpha = np.array([alpha_, - alpha_])

	#定义8组关节角
	Q = np.zeros([8,7])

	#i=0代表上臂形,i=1代表下臂形
	for i in range(2):
		# 求变换矩阵,第二个下标是参考坐标系
		R4_3 = np.array([[math.cos(qq4[i]), 0, math.sin(qq4[i])],
						 [math.sin(qq4[i]), 0, -math.cos(qq4[i])],
						 [0			, 1, 0]])

		#**求取初始R03**#
		#求取y3_0
		y3_0 = -np.dot(bf.euler_axis2rot(u_l, alpha[i]), u_sw)
		#求取x3_0
		x3_0 = (sw + (d3 + d5 * np.cos(qq4[i])) * y3_0) / (d5 * np.sin(qq4[i]))
		#求取z3_0
		z3_0 = np.cross(x3_0,y3_0)
		z3_0 = z3_0/nla.norm(z3_0)

		#求取坐标系3在初始位置下在基座标系中的表示
		R_03_0 = np.zeros([3,3])
		R_03_0[:, 0] = x3_0
		R_03_0[:, 1] = y3_0
		R_03_0[:, 2] = z3_0

		#**求臂型角,基于关节角最小化**#
		#求取去q1,q2,q3对应重点R_psi_mid1
		R_psi_mid1 = np.dot(R_03_mid, R_03_0.T)
		Q_psi_mid1 = bf.rot_to_quaternion(R_psi_mid1)

		#求取去q5,q6,q7对应重点R_psi_mid2
		R_psi_mid2 = np.dot(np.dot(Te[0:3, 0:3], R_74_mid),np.dot(R_03_0, R4_3).T)
		Q_psi_mid2 = bf.rot_to_quaternion(R_psi_mid2)

		#求取四元数中点值
		Q_mid = (Q_psi_mid1 + Q_psi_mid2)/2.0

		#获得优化臂型角
		psi = 0.0
		if (abs(Q_mid[0]) < math.pow(10, -6)):
			psi = -pi
		else:
			psi = math.atan(np.dot(Q_mid[1:4], u_sw)/Q_mid[0])

		# 求取臂型角到到旋转矩阵
		R_psi = bf.euler_axis2rot(u_sw, psi)

		#求取3号坐标系在任意位置在基坐标系中的表示
		R_03 = np.dot(R_psi,R_03_0)

		#求取关节角2,有两组解
		qq2_1 = np.arctan2(np.sqrt(1 - R_03[2, 1] ** 2), -R_03[2,1])
		qq2_2 = np.arctan2(-np.sqrt(1 - R_03[2, 1] ** 2), -R_03[2, 1])

		#求取关节角1
		qq1_1 = np.arctan2(-R_03[1, 1] * np.sin(qq2_1), -R_03[0, 1] * np.sin(qq2_1))
		qq1_2 = np.arctan2(-R_03[1, 1] * np.sin(qq2_2), -R_03[0, 1] * np.sin(qq2_2))

		#求取关节角3
		qq3_1 = np.arctan2(R_03[2, 2] * np.sin(qq2_1), -R_03[2, 0] * np.sin(qq2_1))
		qq3_2 = np.arctan2(R_03[2, 2] * np.sin(qq2_2), -R_03[2, 0] * np.sin(qq2_2))

		#****求取关节角5,6,7****#

		R4 = np.dot(R_psi,np.dot(R_03_0,R4_3))
		R4_7 = np.dot(Te[0:3,0:3].T,R4)
		R_47 = R4_7.T

		#求取关节角6,存在两组解
		qq6_1 = np.arctan2(np.sqrt(1 - R_47[2, 2] ** 2), R_47[2, 2])
		qq6_2 = np.arctan2(-np.sqrt(1 - R_47[2, 2] ** 2), R_47[2, 2])

		# 求取关节角1
		qq5_1 = np.arctan2(R_47[1, 2] * np.sin(qq6_1), R_47[0, 2] * np.sin(qq6_1))
		qq5_2 = np.arctan2(R_47[1, 2] * np.sin(qq6_2), R_47[0, 2] * np.sin(qq6_2))

		# 求取关节角3
		qq7_1 = np.arctan2(R_47[2, 1] * np.sin(qq6_1), -R_47[2, 0] * np.sin(qq6_1))
		qq7_2 = np.arctan2(R_47[2, 1] * np.sin(qq6_2), -R_47[2, 0] * np.sin(qq6_2))

		#求解8组解
		q = np.array([[qq1_1, qq2_1, qq3_1, qq4[i], qq5_1, qq6_1, qq7_1],
					  [qq1_1, qq2_1, qq3_1, qq4[i], qq5_2, qq6_2, qq7_2],
					  [qq1_2, qq2_2, qq3_2, qq4[i], qq5_1, qq6_1, qq7_1],
					  [qq1_2, qq2_2, qq3_2, qq4[i], qq5_2, qq6_2, qq7_2]])
		if(i==0):
			Q[0:4,:] = q
		else:
			Q[4:8,:] = q
	return Q

#采用x0与sw做参考面，求取第二臂形角8组解，时间0.3ms
def arm_angle_ikine_second_limit(DH,Te):
	'''
	:param DH:
	:param psi:
	:param Te:
	:return:
	'''
	#DH参数
	d1 = DH[0, 3]
	d3 = DH[2, 3]
	d5 = DH[4, 3]
	d7 = DH[6, 3]
	#求取SW
	s = np.array([0,0,d1])
	x0 = np.array([1,0,0])
	w = Te[0:3,3] - d7*Te[0:3,2]
	sw = w - s
	l_sw = nla.norm(sw)
	u_sw = sw/l_sw

	#***求取关节角4,第二个参数分别代表q4两组解***#
	#计算beta值
	beta = np.arccos((d3**2 + d5**2 - l_sw**2)/(2*d3*d5))
	#关节角
	qq4_1 = pi - beta #上
	qq4_2 = beta - pi #下
	qq4 = np.array([qq4_1,qq4_2])

	# 求解关节中心点旋转矩阵,由于关节极限对称,所以为0
	R_03_mid = np.array([[1, 0, 0],
						 [0, 0, 1],
						 [0, -1, 0]])

	R_74_mid = np.array([[1, 0, 0],
						 [0, 1, 0],
						 [0, 0, 1]])

	# ***求取关节角1,2,3,第三个参数分别代表q2两组解***#
	#求取夹角alpha
	alpha_ = np.arccos((d3**2 + l_sw**2 - d5**2)/(2*d3*l_sw))
	#定义转轴,用于求y_3_0
	V = x0 #定义参考轴,第二参考轴
	l = np.cross(sw,V) #参考平面法向量
	u_l = l/nla.norm(l)

	#对应上下臂形
	alpha = np.array([alpha_, - alpha_])

	#定义8组关节角
	Q = np.zeros([8,7])

	#i=0代表上臂形,i=1代表下臂形
	for i in range(2):
		# 求变换矩阵,第二个下标是参考坐标系
		R4_3 = np.array([[math.cos(qq4[i]), 0, math.sin(qq4[i])],
						 [math.sin(qq4[i]), 0, -math.cos(qq4[i])],
						 [0		, 1, 0]])

		#求取y3_0
		y3_0 = -np.dot(bf.euler_axis2rot(u_l, alpha[i]), u_sw)
		#求取x3_0
		x3_0 = (sw + (d3 + d5 * np.cos(qq4[i])) * y3_0) / (d5 * np.sin(qq4[i]))
		#求取z3_0
		z3_0 = np.cross(x3_0,y3_0)
		z3_0 = z3_0/nla.norm(z3_0)

		#求取坐标系3在初始位置下在基座标系中的表示
		R_03_0 = np.zeros([3,3])
		R_03_0[:, 0] = x3_0
		R_03_0[:, 1] = y3_0
		R_03_0[:, 2] = z3_0

		# **求臂型角,基于关节角最小化**#
		# 求取去q1,q2,q3对应重点R_psi_mid1
		R_psi_mid1 = np.dot(R_03_mid, R_03_0.T)
		Q_psi_mid1 = bf.rot_to_quaternion(R_psi_mid1)

		# 求取去q5,q6,q7对应重点R_psi_mid2
		R_psi_mid2 = np.dot(np.dot(Te[0:3, 0:3], R_74_mid), np.dot(R_03_0, R4_3).T)
		Q_psi_mid2 = bf.rot_to_quaternion(R_psi_mid2)

		# 求取四元数中点值
		Q_mid = (Q_psi_mid1 + Q_psi_mid2) / 2.0

		# 获得优化臂型角
		psi = 0.0
		if (abs(Q_mid[0]) < math.pow(10, -6)):
			psi = -pi
		else:
			psi = math.atan(np.dot(Q_mid[1:4], u_sw) / Q_mid[0])

		# 求取臂型角到到旋转矩阵
		R_psi = bf.euler_axis2rot(u_sw, psi)

		#求取3号坐标系在任意位置在基坐标系中的表示
		R_03 = np.dot(R_psi,R_03_0)

		#求取关节角2,有两组解
		qq2_1 = np.arctan2(np.sqrt(1 - R_03[2, 1] ** 2), -R_03[2,1])
		qq2_2 = np.arctan2(-np.sqrt(1 - R_03[2, 1] ** 2), -R_03[2, 1])

		#求取关节角1
		qq1_1 = np.arctan2(-R_03[1, 1] * np.sin(qq2_1), -R_03[0, 1] * np.sin(qq2_1))
		qq1_2 = np.arctan2(-R_03[1, 1] * np.sin(qq2_2), -R_03[0, 1] * np.sin(qq2_2))

		#求取关节角3
		qq3_1 = np.arctan2(R_03[2, 2] * np.sin(qq2_1), -R_03[2, 0] * np.sin(qq2_1))
		qq3_2 = np.arctan2(R_03[2, 2] * np.sin(qq2_2), -R_03[2, 0] * np.sin(qq2_2))

		#****求取关节角5,6,7****#
		#求变换矩阵,第二个下标是参考坐标系
		R4_3 = np.array([[np.cos(qq4[i]), 0, np.sin(qq4[i])],
						 [np.sin(qq4[i]), 0, -np.cos(qq4[i])],
						 [0				, 1, 0]])
		R4 = np.dot(R_psi,np.dot(R_03_0,R4_3))
		R4_7 = np.dot(Te[0:3,0:3].T,R4)
		R_47 = R4_7.T

		#求取关节角6,存在两组解
		qq6_1 = np.arctan2(np.sqrt(1 - R_47[2, 2] ** 2), R_47[2, 2])
		qq6_2 = np.arctan2(-np.sqrt(1 - R_47[2, 2] ** 2), R_47[2, 2])

		# 求取关节角1
		qq5_1 = np.arctan2(R_47[1, 2] * np.sin(qq6_1), R_47[0, 2] * np.sin(qq6_1))
		qq5_2 = np.arctan2(R_47[1, 2] * np.sin(qq6_2), R_47[0, 2] * np.sin(qq6_2))

		# 求取关节角3
		qq7_1 = np.arctan2(R_47[2, 1] * np.sin(qq6_1), -R_47[2, 0] * np.sin(qq6_1))
		qq7_2 = np.arctan2(R_47[2, 1] * np.sin(qq6_2), -R_47[2, 0] * np.sin(qq6_2))

		#求解8组解
		q = np.array([[qq1_1, qq2_1, qq3_1, qq4[i], qq5_1, qq6_1, qq7_1],
					  [qq1_1, qq2_1, qq3_1, qq4[i], qq5_2, qq6_2, qq7_2],
					  [qq1_2, qq2_2, qq3_2, qq4[i], qq5_1, qq6_1, qq7_1],
					  [qq1_2, qq2_2, qq3_2, qq4[i], qq5_2, qq6_2, qq7_2]])
		if(i==0):
			Q[0:4,:] = q
		else:
			Q[4:8,:] = q
	return Q

#选择求取关节所需参考面,并调用对应算法求取8组关节角
def arm_angle_all_ikine_limit( DH_0,Te):
	'''
	:param DH_0:
	:param psi:
	:param Te:
	:return:
	'''
	##判断臂形角是否奇异
	d1 = DH_0[0, 3]
	d7 = DH_0[6, 3]
	s = np.array([0, 0, d1])
	w = Te[0:3, 3] - d7 * Te[0:3, 2]
	sw = w - s
	if (np.abs(sw[0] < pow(10, -6)) and np.abs(sw[1] < pow(10, -6))):
		print "臂形角奇异,采用第二臂形角求解"
		#调用第二臂形角计算
		Q = arm_angle_ikine_second_limit(DH_0, Te)
	else:
		Q = arm_angle_ikine_first_limit(DH_0, Te)
	return Q

#基于臂型角逆解运算,单一值输出接口,运行时间0.4ms
def arm_angle_ikine_limit(Te, qk, DH_0, qq_min, qq_max):
	'''
	:param Te:末端矩阵
	:param psi:第一臂形角
	:param qk:上一时刻关节角
	:param DH_0:
	:param q_min:
	:param qq_max:
	:return:关节角和求解标签
	'''
	#建立求解是否成功标签
	succeed_label = True  # 默认成功求逆

	#调用求解函数求取8组解
	Q = arm_angle_all_ikine_limit(DH_0,Te)

	[q,limit_flag] = arm_angle_ikine_chose(Q, qk, qq_min,qq_max)
	if(limit_flag):
		print "求解失败！"
		succeed_label = False

	return [q, succeed_label]

#=================UR构型解析解，DH坐标系需要满足要求=================#
#建立解ms+nc=d的求解函数
def mnsd(m,n,d):
	'''
	:param m:
	:param n:
	:param d:
	:return:
	'''
	a = m**2 + n**2 -d**2
	if(a<0):
		print "不满足求解条件！"
		return [0,0]
	qq1 = np.arctan2(d/np.sqrt(m**2 + n**2), np.sqrt(1 - d**2/(m**2 + n**2))) \
			- np.arctan2(n,m)
	qq2 = np.arctan2(d / np.sqrt(m ** 2 + n ** 2), -np.sqrt(1 - d ** 2 / (m ** 2 + n ** 2))) \
			- np.arctan2(n, m)
	return [qq1,qq2]

#求解T1_inv*Te*T6_inv
def ttt(qq1,qq5,qq6,d1,d5,d6,Te):
	'''
	:param qq1:
	:param qq5:
	:param qq6:
	:param d1:
	:param d5:
	:param d6:
	:param Te:
	:return:
	'''
	#求解T0_1的逆
	T0_1_inv = np.array([[np.cos(qq1), np.sin(qq1), 0, 0],
						 [  		0, 			 0, 1, -d1],
						 [np.sin(qq1), -np.cos(qq1 ),  0, 0],
						 [			0 , 			 0,  0, 1]])

	T5_6_inv = np.array([[np.cos(qq6), np.sin(qq6), 0, 0],
						 [-np.sin(qq6), np.cos(qq6), 0, 0],
						 [			  0,  		   0, 1, -d6],
						 [				0, 			 0, 0, 1]])

	T4_5_inv = np.array([[np.cos(qq5), np.sin(qq5),  0, 0],
						 [			0, 			0,  -1, d5],
						 [-np.sin(qq5), np.cos(qq5), 0, 0],
						 [			0, 			 0,  0, 1]])

	A = np.dot(np.dot(np.dot(T0_1_inv,Te),T5_6_inv),T4_5_inv)
	return A

#求解关节角234
def theta234(A,a2,a3):
	'''
	:param A:
	:param a2:
	:param a3:
	:param d5:
	:return:
	'''
	#求关节角3
	h = (np.power(A[0,3], 2) + np.power(A[1,3], 2) - np.power(a2, 2) - np.power(a3, 2)) / (2 * a2*a3)

	qq3_1 = math.acos(h)
	qq3_2 = -math.acos(h)

	#求关节角2
	s2_1 = ((a3 * math.cos(qq3_1) + a2) * A[1, 3] - a3 * math.sin(qq3_1) * A[0, 3]) / \
		   (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * math.cos(qq3_1))
	c2_1 = (A[0, 3] + a3 * math.sin(qq3_1) * s2_1) / (a3 * math.cos(qq3_1) + a2)

	s2_2 = ((a3 * math.cos(qq3_2) + a2) * A[1, 3] - a3 * math.sin(qq3_2) * A[0, 3]) / \
		   (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * math.cos(qq3_2))
	c2_2 = (A[0, 3] + a3 * math.sin(qq3_2) * s2_2) / (a3 * math.cos(qq3_2) + a2)

	qq2_1 = math.atan2(s2_1, c2_1)
	qq2_2= math.atan2(s2_2, c2_2)

	#关节角4
	qq_234 = math.atan2(A[1, 0], A[0, 0])
	qq4_1 = qq_234 - qq2_1 - qq3_1
	qq4_2 = qq_234 - qq2_2 - qq3_2
	return [[qq2_1, qq3_1, qq4_1], [qq2_2, qq3_2, qq4_2]]

#采用看解析发求逆
def ur_ikine(DH_0,Te):
	'''
	:param DH: 按theta\alpha\a\d排列
	:param Te: n,o,a,p排列
	:return:
	'''
	#获取DH参数
	d1 = DH_0[0, 3]
	a2 = DH_0[1, 2]
	a3 = DH_0[2, 2]
	d4 = DH_0[3, 3]
	d5 = DH_0[4, 3]
	d6 = DH_0[5, 3]

	#******************求解关节1******************#
	#中间参数
	m1 = Te[0,3] - d6*Te[0,2]
	n1 = d6*Te[1,2] - Te[1,3]
	#求取关节角,第二个数值代表关节角1对应两组解的编号
	[qq1_1,qq1_2] = mnsd(m1, n1, d4)

	# ******************求解关节5******************#
	# 中间参数
	m5_1 = np.sin(qq1_1) * Te[0, 2] - np.cos(qq1_1) * Te[1, 2]
	m5_2 = np.sin(qq1_2) * Te[0, 2] - np.cos(qq1_2) * Te[1, 2]
	#求取关节角,第三个数值代表关节角5对应两组解的编号
	qq5_1_1 = np.arctan2(np.sqrt(1 - m5_1 ** 2), m5_1)
	qq5_1_2 = np.arctan2(-np.sqrt(1 - m5_1 ** 2), m5_1)
	qq5_2_1 = np.arctan2(np.sqrt(1 - m5_2 ** 2), m5_2)
	qq5_2_2 = np.arctan2(-np.sqrt(1 - m5_2 ** 2), m5_2)

	# ******************求解关节6******************#
	# 中间参数
	m6_1 = -np.sin(qq1_1) * Te[0, 1] + np.cos(qq1_1) * Te[1, 1]
	n6_1 = np.sin(qq1_1) * Te[0, 0] - np.cos(qq1_1) * Te[1, 0]
	m6_2 = -np.sin(qq1_2) * Te[0, 1] + np.cos(qq1_2) * Te[1, 1]
	n6_2 = np.sin(qq1_2) * Te[0, 0] - np.cos(qq1_2) * Te[1, 0]
	# 求取关节角
	qq6_1_1 = np.arctan2(np.sin(qq5_1_1), 0) - np.arctan2(n6_1, m6_1)
	qq6_1_2 = np.arctan2(np.sin(qq5_1_2), 0) - np.arctan2(n6_1, m6_1)
	qq6_2_1 = np.arctan2(np.sin(qq5_2_1), 0) - np.arctan2(n6_2, m6_2)
	qq6_2_2 = np.arctan2(np.sin(qq5_2_2), 0) - np.arctan2(n6_2, m6_2)
	# ******************求解关节2,3,4******************#
	# 中间参数
	A_1_1 = ttt(qq1_1,qq5_1_1, qq6_1_1, d1, d5, d6, Te)
	A_1_2 = ttt(qq1_1,qq5_1_2, qq6_1_2, d1, d5, d6, Te)
	A_2_1 = ttt(qq1_2,qq5_2_1, qq6_2_1, d1, d5, d6, Te)
	A_2_2 = ttt(qq1_2,qq5_2_2, qq6_2_2, d1, d5, d6, Te)
	# 求取关节角,第四个数值代表关节角2对应两组解的编号
	[qq234_1_1_1, qq234_1_1_2] = theta234(A_1_1, a2, a3)
	[qq234_1_2_1, qq234_1_2_2] = theta234(A_1_2, a2, a3)
	[qq234_2_1_1, qq234_2_1_2] = theta234(A_2_1, a2, a3)
	[qq234_2_2_1, qq234_2_2_2] = theta234(A_2_2, a2, a3)

	# ******************组建8组解******************#
	qq = np.array([[qq1_1, qq234_1_1_1[0], qq234_1_1_1[1], qq234_1_1_1[2], qq5_1_1, qq6_1_1],
				   [qq1_1, qq234_1_1_2[0], qq234_1_1_2[1], qq234_1_1_2[2], qq5_1_1, qq6_1_1],
				   [qq1_1, qq234_1_2_1[0], qq234_1_2_1[1], qq234_1_2_1[2], qq5_1_2, qq6_1_2],
				   [qq1_1, qq234_1_2_2[0], qq234_1_2_2[1], qq234_1_2_2[2], qq5_1_2, qq6_1_2],
				   [qq1_2, qq234_2_1_1[0], qq234_2_1_1[1], qq234_2_1_1[2], qq5_2_1, qq6_2_1],
				   [qq1_2, qq234_2_1_2[0], qq234_2_1_2[1], qq234_2_1_2[2], qq5_2_1, qq6_2_1],
				   [qq1_2, qq234_2_2_1[0], qq234_2_2_1[1], qq234_2_2_1[2], qq5_2_2, qq6_2_2],
				   [qq1_2, qq234_2_2_2[0], qq234_2_2_2[1], qq234_2_2_2[2], qq5_2_2, qq6_2_2]])

	#print "qq:\n",np.around(qq*180/pi)
	#将求解范围转换到[-pi,pi]
	Q = np.zeros([8,6])
	for i in range(8):
		qq[i, :] = qq[i,:] - DH_0[:, 0]
		Q[i,:] = bf.qq_choose(qq[i,:])
	#print "Q:\n", np.around(Q * 180 / pi)
	return Q

#求解唯一解的ur解析解,时间为0.5ms
def ur_ikine_choice(DH_0,Te,qq_k):
	'''
	:param DH: DH参数
	:param Te: 末端齐次位姿
	:param qq_k: 上一时刻关节角
	:return:
	'''
	#求取8组解
	Q = ur_ikine(DH_0, Te)
	#调用选择函数求取最佳关节角
	qq = bf.qq_eight_choice(Q, qq_k)
	return qq

#==========================通用运动学类======================#
class GeneralKinematic(object):
	'''
	函数依赖math和numpy
	'''
	def __init__(self, DH_0,q_min=rp.q_min, q_max=rp.q_max):
		self.DH_0 = DH_0
		self.theta = DH_0[:, 0]
		self.alpha = DH_0[:, 1]
		self.a = DH_0[:, 2]
		self.d = DH_0[:, 3]
		self.q_min = q_min
		self.q_max = q_max

		self.n = len(self.theta)

	#相邻关节传递矩阵
	def trans(self, theta, alpha, a, d):
		T = np.array([[math.cos(theta), -math.sin(theta) * math.cos(alpha),
					   math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
					  [math.sin(theta), math.cos(theta) * math.cos(alpha),
					   -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
					  [0, math.sin(alpha), math.cos(alpha), d],
					  [0, 0, 0, 1]])
		return T

	# ZYX欧拉角转变为旋转矩阵
	def euler_zyx2rot(self,phi):
		'''
			ZYX欧拉角转变为旋转矩阵
			input:欧拉角
			output:旋转矩阵
		'''
		R = np.array([[np.cos(phi[0]) * np.cos(phi[1]),np.cos(phi[0]) * np.sin(phi[1]) * np.sin(phi[2]) - np.sin(phi[0]) * np.cos(phi[2]),
					   np.cos(phi[0]) * np.sin(phi[1]) * np.cos(phi[2]) + np.sin(phi[0]) * np.sin(phi[2])],
					  [np.sin(phi[0]) * np.cos(phi[1]),np.sin(phi[0]) * np.sin(phi[1]) * np.sin(phi[2]) + np.cos(phi[0]) * np.cos(phi[2]),
					   np.sin(phi[0]) * np.sin(phi[1]) * np.cos(phi[2]) - np.cos(phi[0]) * np.sin(phi[2])],
					  [-np.sin(phi[0]), np.cos(phi[1]) * np.sin(phi[2]), np.cos(phi[1]) * np.cos(phi[2])]])
		return R

	# 旋转矩阵转变为ZYX欧拉角
	def rot2euler_zyx(self, Re):
		'''
			ZYX欧拉角速度变为姿态角速度转化矩阵
			input:旋转矩阵
			output:欧拉角[alpha,beta,gamma]
		'''
		euler_zyx = np.zeros(3)
		if (abs(abs(Re[2, 0]) - 1) < math.pow(10, -6)):
			if (Re[2, 0] < 0):
				beta = pi / 2
				alpha = np.arctan2(-Re[1, 2], Re[1, 1])
				gamma = 0
			else:
				beta = -pi / 2
				alpha = -np.arctan2(-Re[1, 2], Re[1, 1])
				gamma = 0
		else:
			p_beta = math.asin(-Re[2, 0])
			cb = np.cos(p_beta)
			alpha = math.atan2(Re[1, 0] * cb, Re[0, 0] * cb)
			gamma = math.atan2(Re[2, 1] * cb, Re[2, 2] * cb)
			if ((math.sin(gamma) * Re[2, 1]) < 0):
				beta = pi - p_beta
			else:
				beta = p_beta
		euler_zyx[0] = alpha
		euler_zyx[1] = beta
		euler_zyx[2] = gamma
		return euler_zyx

	# 将关节角计算到正负pi
	def qq_choose(self, qq):
		'''
			本函数用于选着关节角范围
			input:qq为计算出的关节角
			output:q关节角范围[-pi,pi]
		'''
		q = np.copy(qq)
		for i in range(self.n):
			while (q[i] > math.pi):
				q[i] = q[i] - 2 * math.pi
			while (q[i] < - math.pi):
				q[i] = q[i] + 2 * math.pi
		return q

	#正运动学,返回齐次矩阵
	def fkine(self, qr):
		An = np.eye(4)
		for i in range(self.n):
			T = self.trans(self.theta[i] + qr[i], self.alpha[i], self.a[i], self.d[i])
			An = np.dot(An, T)  # 末端到惯性坐标系传递矩阵
		return An

	#正运动学,输出六维末端位姿,姿态用zyx欧拉角表示
	def fkine_euler(self, qr):
		xe = np.zeros(6)
		An = np.eye(4)
		for i in range(self.n):
			T = self.trans(self.theta[i] + qr[i], self.alpha[i], self.a[i], self.d[i])
			An = np.dot(An, T)  # 末端到惯性坐标系传递矩阵
		xe[0:3] = An[0:3, 3]
		xe[3:6] = self.rot2euler_zyx(An[0:3, 0:3])

		return xe

	#求取雅克比矩阵
	def jeco(self, qr):
		# 计算雅克比矩阵
		U = np.eye(4)
		Jn = np.zeros([6, self.n])
		T = np.zeros([4, 4, self.n])
		for i in range(self.n):
			i = self.n - i - 1
			T[:, :, i] = self.trans(self.theta[i] + qr[i], self.alpha[i], self.a[i], self.d[i])
			U = np.dot(T[:, :, i], U)
			dd = np.array([-U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3],
						   -U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3],
						   -U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3]])
			Jn[0:3, i] = dd
			Jn[3:6, i] = U[2, 0:3]

		An = self.fkine(qr)
		R = An[0:3, 0:3]
		J_R = np.zeros([6, 6])
		J_R[0:3, 0:3] = R
		J_R[3:6, 3:6] = R

		J0 = np.dot(J_R, Jn)
		return J0

	# ***基于雅克比矩阵迭代求解逆运动学***#
	def iterate_ikine(self, q_guess, Te, efs=pow(10, -12), i_max=1000):
		'''
			本函数基于雅克比迭代求解n自由度机械臂逆运动学方程
				 q_ready是上一时刻的位置,单位:弧度;
			     T0e为DH坐标系确定的DH{0}坐标系与DH{6}之间的关系(目标矩阵);
			     efs求解误差阀值，默认值10^(-10)
				 i_limit迭代最大次数,默认值1000
			output:qq为相对与DH_q0的转动角度,单位:弧度;已处理到[-pi, pi] 之间
		'''
		# 建立初时刻迭代初值
		q_r =self.theta + q_guess

		# 计数及标签
		deltaQ = 1
		temp_count = 0

		# 迭代循环求解
		while (deltaQ > efs):

			# 求解正运动学
			An = np.eye(4)
			T = np.zeros([4, 4, self.n])

			for i in range(self.n):
				T[:, :, i] = self.trans(q_r[i], self.alpha[i], self.a[i], self.d[i])
				An = np.dot(An, T[:, :, i])

			# 计算末端误差
			dA = np.zeros(6)
			dA[0:3] = Te[0:3, 3] - An[0:3, 3]
			dA[3:6] = 0.5 * (np.cross(An[0:3, 0], Te[0:3, 0]) + np.cross(An[0:3, 1], Te[0:3, 1])
							 + np.cross(An[0:3, 2], Te[0:3, 2]))

			# 计算雅克比矩阵
			U = np.eye(4)
			Jn = np.zeros([6, self.n])
			for i in range(self.n):
				i = self.n - i - 1
				U = np.dot(T[:, :, i], U)

				dd = np.array([-U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3],
							   -U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3],
							   -U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3]])
				Jn[0:3, i] = dd
				Jn[3:6, i] = U[2, 0:3]

			R = An[0:3, 0:3]
			J_R = np.zeros([6, 6])
			J_R[0:3, 0:3] = R
			J_R[3:6, 3:6] = R

			J0 = np.dot(J_R, Jn)
			# 求取关节角关节角度偏差值
			dq = np.dot(np.linalg.pinv(J0), dA)
			q_r = q_r + dq
			deltaQ = np.linalg.norm(dq)
			temp_count = temp_count + 1
			if (temp_count > i_max):
				print("Solution wouldn't converge")
				return q_guess

		q_tmp = q_r - self.theta
		q = self.qq_choose(q_tmp)
		return q

	#求取UR逆解,输出相对角度
	def ur_ikine(self,Te, qq_k):
		qr = ur_ikine_choice(self.DH_0, Te, qq_k)
		return qr

	#带关节限制
	def iterate_ikine_limit_xyz(self, q_guess, Xe):
		Te = np.eye(4)
		Te[0:3, 0:3] = self.euler_zyx2rot(Xe[3:])
		Te[0:3, 3] = Xe[:3]
		qr = self.iterate_ikine(q_guess, Te)
		flag = bf.exceed_joint_limit(qr ,self.q_min, self.q_max)
		if(flag):
			#print "flag:", flag
			qr = np.copy(q_guess)
		return qr

	# 带关节限制
	def iterate_ikine_limit(self, q_guess, Te):
		qr = self.iterate_ikine(q_guess, Te)
		flag = bf.exceed_joint_limit(qr, self.q_min, self.q_max)
		if (flag):
			# print "flag:", flag
			qr = np.copy(q_guess)
		return qr

def ur_ikine_test():
	DH_0 = rp.DH0_ur5
	#正运动学
	qq = np.array([1,
				   1,
				   1,
				   1,
				   1,
				   1])
	print "qq:", np.around(qq*180/np.pi, 2)
	Te = fkine(DH_0[:, 0] + qq, DH_0[:, 1], DH_0[:, 2], DH_0[:, 3])
	print "Te:\n", np.around(Te, 2)
	Q = ur_ikine(DH_0, Te)
	print "Q:\n", np.around(Q*180/np.pi, 4)
	for i in range(8):
		Te_i = fkine(DH_0[:, 0] + Q[i,:], DH_0[:, 1], DH_0[:, 2], DH_0[:, 3])
		print "Te_",str(i+1),":\n", np.around(Te_i, 2)
		print "Te_", str(i+1), "_error:\n", np.around(Te_i - Te, 2)

def main():
	ur_ikine_test()

if __name__=="__main__":
	main()
