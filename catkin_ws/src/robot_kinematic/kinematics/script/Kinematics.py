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

#=================叉乘操作矩阵=================#
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

#=================欧拉轴角到旋转矩阵rotate=================#
def euler_axis2rot(n,theta):
	'''
		本函数将欧拉角转化为旋转矩阵
		input:n欧拉轴、theta转角
		output:R旋转矩阵
	'''
	n_m = cross_matrix(n)
	R = np.eye(3) + n_m*np.sin(theta) + np.dot(n_m,n_m)*(1 - np.cos(theta))
	return R

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

#=================ZYX欧拉角速度变为姿态角速度转化矩阵=================#
def  J_euler_zyx(Phi):
	'''
		ZYX欧拉角速度变为姿态角速度转化矩阵
		input:欧拉角
		output:角速度变换矩阵
	'''
	J = np.array([[0, -np.sin(Phi[0]), np.cos(Phi[0])*np.cos(Phi[1])],
		[0, np.cos(Phi[0]), np.sin(Phi[0])*np.cos(Phi[1])],
		[1, 0, -np.sin(Phi[1])]])
	return J

#=================ZYX欧拉角转变为旋转矩阵=================#
def  euler_zyx2rot(phi):
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

#=================旋转矩阵转变为ZYX欧拉角=================#
def  rot2euler_zyx(Re):
	'''
		ZYX欧拉角速度变为姿态角速度转化矩阵
		input:旋转矩阵
		output:欧拉角
	'''
	euler_zyx = np.zeros(3)
	if((np.abs(Re[2,1]) < pow(10,-12)) & (np.abs(Re[2,2]) < pow(10,-12))):
		euler_zyx[1] = math.atan2(-Re[2,0],Re[2,2])
		euler_zyx[0] = math.atan2(-Re[0,1],Re[1,1])
		euler_zyx[2] = 0.0
	else:
		euler_zyx[2] = math.atan2(Re[2,1],Re[2,2])
		euler_zyx[1] = math.atan2(-Re[2,0],(np.cos(euler_zyx[2])*Re[2,2] + np.sin(euler_zyx[2])*Re[2,1]))
		euler_zyx[0] = math.atan2(Re[1,0],Re[0,0])
	return euler_zyx

			
#=================建立n自由度机械臂正运动学=================#
def   fkine(theta,alpha,a,d):
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

#=================雅克比矩阵=================#
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
		Ai = np.dot(Ai,trans(theta[i],alpha[i],a[i],d[i]))
		z_i = Ai[0:3,2]
		print i
		print z_i
		p_i = Ai[0:3,3]
		p_in = p_n - p_i
		J[0:3,i] = np.cross(z_i,p_in)
		J[3:6,i] = z_i
	return J

#================迭代雅可比=================#	
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
	
#=================关节角范围选择函数=================#
def q_choose(q):
	'''
		本函数用于选着关节角范围
		input:qq为计算出的关节角
		output:q关节角范围[-pi,pi]
	'''
	pi = np.pi
	n = len(q)
	for i in range(n):
		while (q[i] > pi):
			q[i] = q[i] -2*pi 
		while (q[i] < - 2*pi):
			q[i] = q[i] +2*pi
	return q

#=================基于臂型角逆解运算=================#	
def a_ikine(DH_0,Te,psi):
	'''
		本函数用于求取7自由度SRS无偏执型机械臂逆运动学
		input:DH_0参数，长度单位mm,角度单位red
			  Te,末端位姿齐次矩阵
			  psi,臂型角参数
		output:q,相对与零位角度
	'''
	#DH参数
	d1 = DH_0[0,3]; d3 = DH_0[2,3]; d5 = DH_0[4,3]; d7 = DH_0[6,3]
	##求取S、W值
	S = np.array([0,0,d1])
	p7 = Te[0:3,3]
	z7 = Te[0:3,2]
	W = p7 - d7*z7
	
	##求取与臂型相关参数
	#求取V,w,l
	V = np.array([0,0,1])
	w = W - S
	ww = np.linalg.norm(w)
	l = np.cross(V,w)
	u_l = l/np.linalg.norm(l)
	
	##求取关节角q4
	#求取beta角取值在0 到π
	beta_1 = math.acos((-pow(ww,2) + pow(d3,2) + pow(d5,2))/(2*d3*d5))   
	beta_2 = -beta_1
	
	#求取q4 取值在[-pi,pi]
	q4_1 = np.pi - beta_1       #上臂形
	q4_2 = -np.pi - beta_2       #下臂形
	q4 = np.array([q4_1,q4_2])
	
	#求取alpha，由w转到-y3
	alpha_1 = -math.acos((pow(d3,2) + pow(ww,2) - pow(d5,2))/(2*d3*ww))  #上臂形
	alpha_2 = -alpha_1                                                   #下臂形
	alpha = np.array([alpha_1,alpha_2])
	
	##求取去，其他关节角度
	q1 = np.zeros(4)
	q2 = np.zeros(4)
	q3 = np.zeros(4)
	q5 = np.zeros(4)
	q6 = np.zeros(4)
	q7 = np.zeros(4)
	for i in range(2):
		#求取欧拉角到旋转矩阵
		R_l_a = euler_axis2rot(u_l,alpha[i])
		
		#求取参考面上的传递参数R0_3_0
		y0_3_0 = -np.dot(R_l_a,w)/ww
		x0_3_0 = (w + (d3 + d5*np.cos(q4[i]))*y0_3_0)/(d5*np.sin(q4[i]))
		z0_3_0 = -np.cross(y0_3_0,w)/(d5*np.sin(q4[i]))         #?
		
		R0_3_0 = np.array([x0_3_0,y0_3_0,z0_3_0]).T
		
		###求取关节角q1,q2,q3
		##通过臂形角求取R0_3的旋转矩阵
		u_w = w/ww
		u_w_m = cross_matrix(u_w)
		#求取0坐标系下臂型面绕w转动phi的转动矩阵
		R0_phi = euler_axis2rot(u_w,psi)
		
		R0_3 = np.dot(R0_phi,R0_3_0)
		
		#求取q2
		if i == 1:
			q2[i+1] = math.acos(-R0_3[2,1])
			q2[i+2] = -q2[i+1]
		else:
			q2[i] = math.acos(-R0_3[2,1])
			q2[i+1] = -q2[i]
		#求取q1,q3
		for j in range(2):
			if i ==1:
				q1[i+j+1] = math.atan2(-R0_3[1,1]*np.sin(q2[i+j+1]),-R0_3[0,1]*np.sin(q2[i+j+1]))
				q3[i+j+1] = math.atan2(R0_3[2,2]*np.sin(q2[i+j+1]),-R0_3[2,0]*np.sin(q2[i+j+1]))
			else:
				q1[i+j] = math.atan2(-R0_3[1,1]*np.sin(q2[i+j]),-R0_3[0,1]*np.sin(q2[i+j]))
				q3[i+j] = math.atan2(R0_3[2,2]*np.sin(q2[i+j]),-R0_3[2,0]*np.sin(q2[i+j]))
	
		###求取关节角q5,q6,q7
		#3到4的旋转矩阵R3_4
		R3_4 = np.array([[math.cos(q4[i]), 0, math.sin(q4[i])],
						[math.sin(q4[i]),  0, -math.cos(q4[i])],
						[0, 1, 0,]])
						
		#求取4到7的旋转矩阵R4_7
		R7_4 = np.dot(np.dot(Te[0:3,0:3].T,R0_phi),np.dot(R0_3_0,R3_4))
		R4_7 = R7_4.T
		
		#求取q6
		if i == 1:
			q6[i+1] = math.acos(R4_7[2,2])
			q6[i+2] = -q6[i+1]
		else:
			q6[i] = math.acos(R4_7[2,2])
			q6[i+1] = -q6[i]
		#求取求，q5,q7
		for j in range(2):
			if i ==1:
				q5[i+j+1] = math.atan2(R4_7[1,2]*np.sin(q6[i+j+1]),R4_7[0,2]*np.sin(q6[i+j+1]))
				q7[i+j+1] = math.atan2(R4_7[2,1]*np.sin(q6[i+j+1]),-R4_7[2,0]*np.sin(q6[i+j+1]))
			else:
				q5[i+j] = math.atan2(R4_7[1,2]*np.sin(q6[i+j]),R4_7[0,2]*np.sin(q6[i+j]))
				q7[i+j] = math.atan2(R4_7[2,1]*np.sin(q6[i+j]),-R4_7[2,0]*np.sin(q6[i+j]))
	
	#整合8组解
	Q = np.array([[q1[0], q1[0], q1[1], q1[1], q1[2], q1[2], q1[3], q1[3]],
				  [q2[0], q2[0], q2[1], q2[1], q2[2], q2[2], q2[3], q2[3]],
				  [q3[0], q3[0], q3[1], q3[1], q3[2], q3[2], q3[3], q3[3]],
				  [q4[0], q4[0], q4[0], q4[0], q4[1], q4[1], q4[1], q4[1]],
				  [q5[0], q5[1], q5[0], q5[1], q5[2], q5[3], q5[2], q5[3]],
				  [q6[0], q6[1], q6[0], q6[1], q6[2], q6[3], q6[2], q6[3]],
				  [q7[0], q7[1], q7[0], q7[1], q7[2], q7[3], q7[2], q7[3]]])
				  
	return Q
	
#=================基于臂型角逆解运算,结果选择=================#	
def analysis_ikine_chose(Q,qr_init,q_limit):
	'''
		本函数用于选择臂形角参数化求逆获得的8组参数的解
		input:Q 臂形角参数化求逆获得的8组参数的解
			  qk 上一时刻的关节角
		output:q选择出的一组关节角，已处理到[-pi, pi] 之间
	'''
	n = len(qr_init)
	dq = np.zeros(8)
	qk = np.copy(qr_init)
	dqi_max = np.zeros([n,8])
	for j in range(8):
		if (bf.exceed_joint_limit(Q[:,j],q_limit)):
			dq[j] = 1000.0 + j
		else:
			dqi_max[:,j] = Q[:,j] - qk
			dq[j] = nla.norm(dqi_max[:,j])
	dq_min = min(dq)
	dq_minIndex = np.where(dq_min == dq_min)
	q = np.zeros(n)
	for i in range(n):
		q[i] = np.copy(Q[i,dq_minIndex])
	return q
	
#=================基于臂型角逆解运算,单一值输出接口=================#	
def aa_ikine(Te,psi,qr_init,q_limit = q_limit_armc,DH_0 = DH0_armc):
	'''
		本函数用于求取7自由度SRS无偏执型机械臂逆运动学
		input:DH_0参数，长度单位mm,角度单位red
			  Te,末端位姿齐次矩阵
			  psi,臂型角参数,可调节
			  qr_init,参考初始角
			  q_limit，关节极限
		output:q,相对与零位角度
			   succeed_label，是否求解成功标签，成功为1，不成功为0
	'''
	succeed_label = 1      #默认成功求逆
	
	current_psi = np.copy(psi)
	current_neg_psi = np.copy(psi)
	current_pos_psi = np.copy(psi)
	
	delt_psi = pi/360.0
	
	Q = a_ikine(DH_0,Te,current_psi)
	q = analysis_ikine_chose(Q,qr_init,q_limit)
	if(bf.exceed_joint_limit(q,q_limit)):
		for i in range(360):
			if (i/2 == 0):   #正方向查找
				current_psi = current_pos_psi + delt_psi
				if(current_psi >= pi):
					current_psi = current_psi - 2*pi  #跨越极限
				current_pos_psi = current_psi
			else:           #负方向查找
				current_psi = current_neg_psi - delt_psi
				if(current_psi < -pi):
					current_psi = current_psi + 2*pi
				current_neg_psi = current_psi
				
			Q = a_ikine(DH_0,Te,current_psi)
			q = analysis_ikine_chose(Q,qr_init,q_limit)
			if((not bf.exceed_joint_limit(q,q_limit)) and (not bf.joint_have_jump(q,qr_init))):
				break
			if(i == 359 and bf.exceed_joint_limit(q,q_limit)):
				succeed_label = 0
				print '给定末端不在工作空间！'
	return [q,current_psi,succeed_label]
	
		
#================基于雅克比矩阵迭代求解逆运动学=================#	
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
	LimitFlag = 0
	n = len(q_r)
	deltaQ = 1  
	temp_count = 0 
	E = np.eye(4)  
	
	#迭代循环求解
	while (deltaQ > efs) & (LimitFlag == 0):
		
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
			q_tmp = q_r - DH_0[:,0]
			q = q_choose(q_tmp)
			LimitFlag = 1
			print("Solution wouldn't converge")
	q_tmp = q_r - DH_0[:,0]		
	q = q_choose(q_tmp)
	return q


