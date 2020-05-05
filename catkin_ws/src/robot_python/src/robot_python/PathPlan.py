#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于求7自由度SRS型机械臂运动规划相关函数
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.9.23
import numpy as np
from math import pi
import numpy.linalg as nla
import math
#自定义函数
import Kinematics as kin
from RobotParameter import DH0_armc
from RobotParameter import q_limit_armc
import BaseFunction as bf
import GeneticAlgorithm as ga

#=================基于位置级求逆圆轨迹规划=================#
def circle_space(DH_0,rc):
	'''
		本函数用于规划圆轨迹
		input:q0,qv0,qa0起点位置速度加速度
			  qf,qvf,qaf终点位置速度加速度
			  tf终点时间，dt时间间隔
		output:Q[n,q_qav,t] = [q,qv,qa] 对应时刻位置速度加速度
	'''
	#规划起点关节角
	q_init = np.array([0,-30, 0, 90, 0 , 30 , 0])*(np.pi/180)
	n = len(q_init)
	theta = DH_0[:,0] + q_init
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	T_init = kin.fkine(theta,alpha,a,d)
	
	#建立规划空间坐标系，默认规划空间坐标系与初始位置7关节坐标系相同，起始点为（-rc,0,0)
	Tb_c = np.array([[1,0,0,rc],
					 [0,1,0,0 ],
					 [0,0,1,0 ],
					 [0,0,0,1]])
	T0_c = np.dot(T_init,Tb_c)
	
	##规划空间建立圆坐标点
	#设置圆的规划点个数,取点频率为100Hz
	nodeNum = 3600
	phi = np.zeros([3,nodeNum])
	
	#前20个点为加速点
	phi_b0 = 0; phi_b0v = 0; phi_b0a = 0;  #调用5次规划求取
	phi_b = np.pi/90.0; phi_bv = np.pi/18.0; phi_ba = 0;
	tf = 0.2
	dt = 0.01
	Phi_b = bf.interp5rdPoly1(phi_b0,phi_b0v,phi_b0a,phi_b,phi_bv,phi_ba,tf,dt)
	phi[:,0:1] = Phi_b[:,0:1]
	
	#20到3500匀速规划点
	phi[0,1:3580] = np.linspace(2*np.pi*(1/3600.0),2*np.pi*35.80/36.0,3579)
	phi[1,1:3580] = np.full(3579,phi_bv)
	#最后100个点为减速点
	phi_e0 = 2*np.pi*(35.8/36.0 + 1/3600.0); phi_e0v = np.pi/18.0; phi_e0a = 0;  #调用5次规划求取
	phi_e = 2*np.pi; phi_ev = 0; phi_ea = 0;
	Phi_e = bf.interp5rdPoly1(phi_e0,phi_e0v,phi_e0a,phi_e,phi_ev,phi_ea,tf,dt)
	phi[:,3580:nodeNum] = Phi_e
	
	#末端圆在规划空间笛卡尔坐标点
	Xc_c = np.ones([4,nodeNum])
	for i in range(nodeNum):
		Xc_c[0,i] = rc*np.cos(phi[0,i])
		Xc_c[1,i] = rc*np.sin(phi[0,i])
		Xc_c[2,i] = 0
	
	#求取逆解，默认姿态保持不变
	q = np.zeros([7,nodeNum])
	q_ready = q_init
	T0_c_i = np.zeros([4,4])
	for i in range(nodeNum):
		T0_c_i[0:3,0:3] = T0_c[0:3,0:3]
		X0_c = np.dot(T0_c,Xc_c[:,i])
		T0_c_i[0:4,3] = X0_c
		q[:,i] = kin.iterate_ikine(DH_0,q_ready,T0_c_i)
		#print kin.fkine(q[:,i],alpha,a,d)
		q_ready = q[:,i]
		
	return q
	
#=================基于速度级求逆直线运动轨迹规划=================#
def P2P_plan(DH_0,l):
	'''
		本函数用于规划圆轨迹
		input:q0,qv0,qa0起点位置速度加速度
			  qf,qvf,qaf终点位置速度加速度
			  tf终点时间，dt时间间隔
		output:Q[n,q_qav,t] = [q,qv,qa] 对应时刻位置速度加速度
	'''
	#规划起点关节角
	qr_init = np.array([0,-30, 0, 90, 0 , 30 , 0])*(np.pi/180)
	n = len(qr_init)
	q_init = DH_0[:,0] + qr_init
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	T_init = kin.fkine(q_init,alpha,a,d)	
	
	#规划末端起点和终点,采用ZYX欧拉角描述角度
	Xe0 = np.zeros(6)
	Xe0[0:3] = T_init[0:3,3]
	Xe0[3:6] = bf.Tr2ZYX(T_init[0:3,0:3])
	
	Xed = np.zeros(6)
	Xed[0:3] = T_init[0:3,3] + np.array([l,0,0])
	Xed[3:6] = Xe0[3:6]#np.array([0,np.pi/2.0,np.pi/2.0])
	
	X_seq = np.zeros([2,6])
	X_seq[0,:] = Xe0
	X_seq[1,:] = Xed
	
	#规划时间及规划点采样时间
	tf = 100.0
	t_seq = np.array([0,tf])
	sampleTime = 0.025
	
	#求取末端三次样条曲线参数
	M = np.zeros([2,6])
	h = t_seq[1] - t_seq[0]
	for i in range(6):
		M[:,i] = bf.spline_param(t_seq,X_seq[:,i],0,0)
	print M
	#采样点求关节空间关节角速度\角度
	k = np.floor(tf/sampleTime).astype(int) + 1

	Q_vel = np.zeros([n,k])
	Q_q = np.zeros([n,k])
	X_xva = np.zeros([3,6,k])       #存储当前点的末端的位置、速度、加速度
	qr_m = qr_init
	
	for i in range(k):
		current_time = i*sampleTime
		#三次样条曲线求取末端笛卡尔空间变量位置、速度、加速度
		for j in range(6):
			X_xva[:,j,i] = bf.spline_tt1(t_seq,X_seq[:,j],current_time,M[:,j],h)
		X_x = X_xva[0,:,i]
		X_v = X_xva[1,:,i]
		
		#求取姿态角速度
		W_xyz = np.dot(kin.J_euler_zyx(X_x[3:6]),X_v[3:6])
		Xw_m = X_v
		Xw_m[3:6] = W_xyz
		
		#求取雅克比矩阵
		Jaco_m = kin.jeco_0(DH_0,qr_m) 
		pinv_jaco = np.linalg.pinv(Jaco_m)
		
		#求取关节角速度
		Q_vel[:,i] = np.dot(pinv_jaco,Xw_m)
		
		#求取关节角度
		Q_q[:,i] = qr_m + sampleTime*Q_vel[:,i]
		
		#更新关节角度
		qr_m = Q_q[:,i]
		
	return [Q_q,Q_vel,X_xva]

#=================多点轨迹规划：直线规划=================#
def multipoint_plan(DH_0,l):
	'''
		本函数用于规划圆轨迹
		input:q0,qv0,qa0起点位置速度加速度
			  qf,qvf,qaf终点位置速度加速度
			  tf终点时间，dt时间间隔
		output:Q[n,q_qav,t] = [q,qv,qa] 对应时刻位置速度加速度
	'''
	#规划起点关节角
	qr_init = np.array([0,-30, 0, 90, 0 , 30 , 0])*(np.pi/180)
	n = len(qr_init)
	q_init = DH_0[:,0] + qr_init
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	T_init = kin.fkine(q_init,alpha,a,d)	
	
	#规划末端起点和终点,采用ZYX欧拉角描述角度
	Xe0 = np.zeros(6)
	Xe0[0:3] = T_init[0:3,3]
	Xe0[3:6] = bf.Tr2ZYX(T_init[0:3,0:3])
	
	Xed = np.zeros(6)
	Xed[0:3] = T_init[0:3,3] + np.array([l,0,0])
	Xed[3:6] = Xe0[3:6]#np.array([0,np.pi/2.0,np.pi/2.0])
	
	#虚拟出关节点
	kk = 10
	phi_t = np.linspace(0,np.pi/2,kk)
	X_seq = np.zeros([6,kk])
	for i in range(kk):
		X_seq[:,i] = Xe0 + (Xed - Xe0)*np.sin(phi_t[i])

	#规划时间及规划点采样时间
	tf = 100.0
	t_seq = np.linspace(0,100.0,kk)
	sampleTime = 0.25
	
	####可以独立出来做多点轨迹规划####
	#末端三次样条曲线末端插补
	ke = np.floor(t_seq[kk-1]/sampleTime).astype(int) + 1
	s = np.zeros([6,ke])
	for i in range(6):
		[s[i,:],vel_,acc_] = bf.spline1(t_seq,X_seq[i,:],sampleTime,0,0)
		
	#位置级求逆
	q_ready = qr_init
	T0e = np.eye(4)
	q = np.zeros([n,ke])
	for i in range(ke):
		R = bf.euler_zyx2rot(s[3:6,i])
		T0e[0:3,0:3] = R
		T0e[0:3,3] = s[0:3,i]
		#print T0e
		q[:,i] = kin.iterate_ikine(DH_0, q_ready, T0e)
		q_ready = q[:,i]
		
	#关节空间采用三次样条曲线插补
	interval = 0.025
	kq = np.floor(t_seq[kk-1]/interval).astype(int) + 1
	tt_seq = np.linspace(0,100.0,ke)
	qq = np.zeros([n,kq])
	qv = np.zeros([n,kq])
	qa = np.zeros([n,kq])
	for i in range(n):
		[qq[i,:],qv[i,:],qa[i,:]] = bf.spline1(tt_seq,q[i,:],interval,0,0)

	return [qq,vel_,acc_]
	
#=================多点轨迹跟踪：位置级求逆，关节空间插补=================#
def multipoint_plan_position(qr_init,X0_e,T,DH_0 = DH0_armc,kk = 10):
	'''
		本函数多点轨迹跟踪：位置级求逆，关节空间插补
		input:[q_init,规划的位置起点
				X0_e,末端位姿点，姿态采用Euler_ZYX描述
				T，末端点采样周期
				DH_0 = DH0_armc] DH参数，默认自制机械臂armc
				kk关节空间插值倍数
		output:[qq,qv,qa]关节位置，速度，加速度
	'''
	#DH参数
	n = len(qr_init)
	theta0 = DH_0[:,0]
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
		
	#位置级求逆
	q_ready = qr_init
	T0e = np.eye(4)
	ke = len(X0_e[:,0])
	q = np.zeros([ke,n])
	for i in range(ke):
		#求取末端齐次矩阵
		R = bf.euler_zyx2rot(X0_e[i,3:6])
		T0e[0:3,0:3] = R
		T0e[0:3,3] = X0_e[i,0:3]
		#数值法求逆
		#q[i,:] = kin.iterate_ikine(DH_0, q_ready, T0e)
		q[i, :] = kin.grad_iterate_ikine(DH_0, q_ready, T0e)             #梯度法
		q_ready = q[i,:]
	#关节空间采用三次样条曲线插补,10倍频率插补
	interval = T/kk
	kq = kk*(ke - 1) + 1
	tt_seq = np.linspace(0,(ke - 1)*T,ke)
	qq = np.zeros([kq,n])
	qv = np.zeros([kq,n])
	qa = np.zeros([kq,n])
	for i in range(n):
		[qq[:,i],qv[:,i],qa[:,i]] = bf.spline1(tt_seq,q[:,i],interval,0,0)

	return [qq,qv,qa]

#=================多点轨迹跟踪：笛卡尔空间插补，速度级求逆，关节空间积分=================#
def multipoint_plan_velocity(qr_init,X0_e,T,DH_0 = DH0_armc ):
	'''
		本函数多点轨迹跟踪：位置级求逆，关节空间插补
		input:[q_init,规划的位置起点
				X0_e,末端位姿点，姿态采用Euler_ZYX描述
				T，末端点采样周期
				DH_0 = DH0_armc] DH参数，默认自制机械臂armc
		output:[qq,qv]关节位置，速度
	'''
	#DH参数
	n = len(qr_init)
	theta0 = DH_0[:,0]
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
		
	#笛卡尔空间末端位姿插补
	ke = len(X0_e[:,0])
	
	#求取末端三次样条曲线参数
	M = np.zeros([ke,6])
	h = np.zeros([ke-1,6])
	t_seq = np.linspace(0,(ke - 1)*T,ke)
	for i in range(6):
		[M[:,i],h[:,i]] = bf.spline_param(t_seq,X0_e[:,i],0,0)
	
	#笛卡尔空间10倍频率采样
	kke = 10*(ke - 1) + 1
	sampleTime = T/10.0
	qv = np.zeros([n,kke])
	qq = np.zeros([n,kke])
	
	qr_m = qr_init #给定积分初值
	qq[:,0] = qr_init
	
	#笛卡尔空间采用三次样条曲线插值
	X_x = np.zeros([kke,6])
	X_v = np.zeros([kke,6])
	X_a = np.zeros([kke,6])
	for i in range(6):
		[X_x[:,i],X_v[:,i],X_a[:,i]] = bf.spline1(t_seq,X0_e[:,i],sampleTime ,0,0)
	#X_x = np.zeros(6)
	#X_v = np.zeros(6)
	for i in range(kke):
		current_time = i*sampleTime
		#三次样条曲线求取末端笛卡尔空间变量位姿、速度
	#	for j in range(6):
	#		[X_x[j],X_v[j],x_a] = spline_tt(t_seq,X0_e[:,j],current_time,M[:,j],h[:,j])
		
		#求取姿态角速度
		W_xyz = np.dot(kin.J_euler_zyx(X_x[i,3:6]),X_v[i,3:6])
		Xw_m = X_v[i,:]
		Xw_m[3:6] = W_xyz
		
		#求取雅克比矩阵
		Jaco_m = kin.jeco_0(DH_0,qr_m) 
		pinv_jaco = np.linalg.pinv(Jaco_m)
		
		#求取关节角速度
		qv[:,i] = np.dot(pinv_jaco,Xw_m)
		
		#积分求取关节角度
		if i != (kke -1):
			qq[:,i + 1] = qr_m + sampleTime*qv[:,i]
			#更新关节角度
			qr_m = qq[:,i + 1]
		
	return [qq,qv,X_a]

#=================基于臂型角的零空间运动规划=================#
def null_space_plan(qr_init,Te,q_limit = q_limit_armc ,DH_0 = DH0_armc):
	'''
		本函数多点轨迹跟踪：位置级求逆，关节空间插补
		input:[q_init,规划的位置起点
				Te，末端姿态矩阵
				DH_0 = DH0_armc] DH参数，默认自制机械臂armc
		output:qq关节位置
	'''
	#DH参数
	n = len(qr_init)
	theta0 = DH_0[:,0]
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	
	#臂形角规划
	T = 0.1
	nodeNum = 381
	psi = np.zeros(nodeNum)
	t_seq = np.linspace(0,38,nodeNum)
	#加速阶段,采用5次多项式插值   (平滑处理）
	psi_b0 = 0;psiv_b0 = 0;psia_b0 = 0;
	psi_bf = pi/18; psiv_bf = pi/18; psia_bf = 0;
	tf = 2; dt = T;
	[psi[0:21],v_,a_] = bf.interp5rdPoly1(psi_b0,psiv_b0,psia_b0,psi_bf,psiv_bf,psia_bf,tf,dt)
	#匀速阶段
	psi[21:360] = np.linspace(pi/18 + pi/180,35*pi/18 - pi/180,339)
	#减速速阶段,采用5次多项式插值
	psi_e0 = 35*pi/18;psiv_e0 = pi/18;psia_e0 = 0;
	psi_ef = 2*pi; psiv_ef = 0; psia_ef = 0;
	tf = 2; dt = T;
	[psi[360:381],v_,a_] = bf.interp5rdPoly1(psi_e0,psiv_e0,psia_e0,psi_ef,psiv_ef,psia_ef,tf,dt)
	
	#臂型角取值范围[-pi,pi)
	psi[-1] = 0
	psi = psi - pi*np.ones(nodeNum)
	#采用臂形角求逆
	q = np.zeros([nodeNum,n])
	dq = np.zeros(8)
	qk = np.copy(qr_init)
	for i in range(nodeNum):
		Q = kin.a_ikine(DH_0,Te,psi[i])
		q[i,:] = kin.analysis_ikine_chose(Q,qk,q_limit)
		qk = np.copy(q[i,:])
	#由最后一点有突变，采用临时解决办法
	q[-1,:] = q[-2,:]
	#关节空间采用三次样条曲线插补
	interval = 0.01
	kq = np.floor(t_seq[nodeNum-1]/interval).astype(int) + 1
	qq = np.zeros([kq,n])
	qv = np.zeros([kq,n])
	qa = np.zeros([kq,n])
	for i in range(n):
		[qq[:,i],qv[:,i],qa[:,i]] = bf.spline1(t_seq,q[:,i],interval,0,0)
		
	return [qq,qv,qa]

#=================多点轨迹跟踪：位置级求逆，关节空间插补,基于臂型角=================#
def multipoint_plan_position_aa(qr_init,X0_e,T,q_limit = q_limit_armc,DH_0 = DH0_armc,kk = 10):
	'''
		本函数多点轨迹跟踪：位置级求逆，关节空间插补
		input:[q_init,规划的位置起点
				X0_e,末端位姿点，姿态采用Euler_ZYX描述
				T，末端点采样周期
				q_limit = q_limit_armc，关节极限，默认自制机械臂armc
				DH_0 = DH0_armc] DH参数，默认自制机械臂armc
				kk关节空间插值倍数
		output:[qq,qv,qa]关节位置，速度，加速度
	'''
	#DH参数
	n = len(qr_init)
	theta0 = DH_0[:,0]
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
		
	#给定初始角度
	q_ready = qr_init
	psi = bf.arm_angle_by_joint(qr_init,DH_0)   #臂型角初始值设为0
	current_psi = psi  #如果有期望臂角，应该代码
	#位置级求逆
	T0e = np.eye(4)
	ke = len(X0_e[:,0])
	q = np.zeros([ke,n])
	for i in range(ke):
		#求取末端齐次矩阵
		R = bf.euler_zyx2rot(X0_e[i,3:6])
		T0e[0:3,0:3] = R
		T0e[0:3,3] = X0_e[i,0:3]
		
		#采用臂形角求逆
		#[current_q,current_psi,succeed_label] = kin.aa_ikine(T0e,current_psi,qr_init,q_limit,DH_0)
		[current_q, current_psi, succeed_label] = kin.armc_ikine(DH_0, T0e, current_psi, qr_init)
		print current_psi 
		q[i,:] = current_q
	#关节空间采用三次样条曲线插补,10倍频率插补
	interval = T/kk
	kq = kk*(ke - 1) + 1
	tt_seq = np.linspace(0,(ke - 1)*T,ke)
	qq = np.zeros([kq,n])
	qv = np.zeros([kq,n])
	qa = np.zeros([kq,n])
	for i in range(n):
		[qq[:,i],qv[:,i],qa[:,i]] = bf.spline1(tt_seq,q[:,i],interval,0,0)

	return [qq,qv,qa]


# =================多点轨迹跟踪：位置级求逆，关节空间插补,基于遗传算法优化的臂型角=================#
def multipoint_plan_position_FGA(qr_init, X0_e, T, q_limit=q_limit_armc, DH_0=DH0_armc, kk=10):
	'''
		本函数多点轨迹跟踪：位置级求逆，关节空间插补
		input:[q_init,规划的位置起点
				X0_e,末端位姿点，姿态采用Euler_ZYX描述
				T，末端点采样周期
				q_limit = q_limit_armc，关节极限，默认自制机械臂armc
				DH_0 = DH0_armc] DH参数，默认自制机械臂armc
				kk关节空间插值倍数
		output:[qq,qv,qa]关节位置，速度，加速度
	'''
	# DH参数
	n = len(qr_init)
	thetaMin = q_limit[0,:]
	thetaMax = q_limit[1,:]

	# 给定初始角度
	psi = bf.arm_angle_by_joint(qr_init,DH_0)  # 臂型角初始值设为0
	current_psi = psi  # 如果有期望臂角，应该代码
	current_q = qr_init

	# 位置级求逆
	T0e = np.eye(4)
	ke = len(X0_e[:, 0])
	q = np.zeros([ke, n])
	Psi = np.zeros(ke)
	for i in range(ke):
		# 求取末端齐次矩阵
		R = bf.euler_zyx2rot(X0_e[i, 3:6])
		T0e[0:3, 0:3] = R
		T0e[0:3, 3] = X0_e[i, 0:3]

		# 采用遗传算法优化臂形角求逆
		#print "current_psi: %s" % current_psi
		#print "current_q: %s" % current_q
		[current_q, current_psi] = ga.genetic_armc_angle(T0e,current_psi,current_q,DH_0,thetaMax ,thetaMin )
		q[i, :] = current_q
		Psi[i] = current_psi

	# 关节空间采用三次样条曲线插补,10倍频率插补
	interval = T / kk
	kq = kk * (ke - 1) + 1
	tt_seq = np.linspace(0, (ke - 1) * T, ke)
	qq = np.zeros([kq, n])
	qv = np.zeros([kq, n])
	qa = np.zeros([kq, n])
	for i in range(n):
		[qq[:, i], qv[:, i], qa[:, i]] = bf.spline1(tt_seq, q[:, i], interval, 0, 0)

	return [qq, qv, qa,Psi]

# =================多点轨迹跟踪：加权最小范数法位置级求逆，关节空间插补=================#
def multipoint_plan_position_w(qr_init, X0_e, T, DH_0,q_max,q_min,kk=10):
	'''
		本函数多点轨迹跟踪：位置级求逆，关节空间插补
		input:[q_init,规划的位置起点
				X0_e,末端位姿点，姿态采用Euler_ZYX描述
				T，末端点采样周期
				DH_0，DH参数，默认自制机械臂armc
				kk关节空间插值倍数
		output:[qq,qv,qa]关节位置，速度，加速度
	'''
	# 参数初值
	n = len(qr_init)
	d_h_k = np.zeros(n)#最优函数偏导
	q_ready = qr_init
	T0e = np.eye(4)
	ke = len(X0_e[:, 0])
	q = np.zeros([ke, n])

	# 位置级求逆
	for i in range(ke):
		# 求取末端齐次矩阵
		R = bf.euler_zyx2rot(X0_e[i, 3:6])
		T0e[0:3, 0:3] = R
		T0e[0:3, 3] = X0_e[i, 0:3]
		# 数值法求逆
		q[i, :], d_h = kin.w_ikine(DH_0,q_ready,q_max,q_min,d_h_k,T0e)   #加权最小范数求逆
		q_ready = q[i, :]
		d_h_k = np.copy(d_h)
	# 关节空间采用三次样条曲线插补,10倍频率插补
	interval = T / kk
	kq = kk * (ke - 1) + 1
	tt_seq = np.linspace(0, (ke - 1) * T, ke)
	qq = np.zeros([kq, n])
	qv = np.zeros([kq, n])
	qa = np.zeros([kq, n])
	for i in range(n):
		[qq[:, i], qv[:, i], qa[:, i]] = bf.spline1(tt_seq, q[:, i], interval, 0, 0)

	return [qq, qv, qa]