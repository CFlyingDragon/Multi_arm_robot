#!/usr/bin/env python
#-*-coding:utf-8-*-
'''
	功能：通过moveit API驱动机械臂,本程序通过python写一个样板程序
	作者：飞龙（CYT)
	时间：2019.8.19
'''
import numpy as np

class ImpedanceControl(object):
	'''
		功能：设计一个阻抗控制基类
	'''
	def __init__(self,DH0,q_max,q_min):
		'''
			功能：初始化属性
			参数：DH0 = [theta,alpha,a,d]
				 q_max
				 q_min
		'''
		self.DH0 = DH0
		self.q_max = q_max
		self.q_min = q_min

	#相邻关节齐次变换矩阵
	def trans(self,theta_,alpha_,a_,d_):
		'''
			功能：求解相邻关节齐次传递矩阵
			输入参数：相邻关节的的DH参数
			返回参数：相邻关节齐次矩阵
		'''

		T = np.array([[np.cos(theta_), -(np.cos(alpha_))*(np.sin(theta_)),(np.sin(alpha_))*(np.sin(theta_)),a*(np.cos(theta_))],
				[ np.sin(theta_), np.cos(alpha_)*(np.cos(theta_)),-(np.sin(alpha_))*(np.cos(theta_)),a*(np.sin(theta_))],
				[ 0, np.sin(alpha_), np.cos(alpha_), d],[0, 0, 0, 1]])
		return T

	#建立n自由度机械臂正运动学
	def fkine(self,q):
		'''
			功能：求取n自由度机械臂正运动学
			输入参数：相对零位关节角度q
			返回参数：0坐标系到n坐标系传递矩阵
		'''
		theta = self.DH0[:,0] + q
		alpha = self.DH0[:1]
		a = self.DH0[:2]
		d = self.DH0[:3]
		#关节自由度
		n = len(theta)
		#建立4×4的齐次传递矩阵,定义为numpy类型
		An = np.eye(4)
		for i  in range(n):
			T =  self.trans(theta[i],alpha[i],a[i],d[i])
			An = np.dot(An,T)  #末端到惯性坐标系传递矩阵
		self.An = An
		return An

	#建立关节角范围选择函数
	def q_choose(self,qq):
		'''
			功能：用于选择关节角范围
			输入参数: qq为计算出的关节角
			返回参数：关节角范围[-pi,pi]
		'''
		pi = np.pi
		q = qq
		n = len(q)
		for i in range(n):
			while (q[i] > pi):
				q[i] = q[i] -2*pi 
			while (q[i] < - 2*pi):
				q[i] = q[i] +2*pi
		return q
	
	#建立n自由度机械臂逆运动学方程
	def ikine(self,qr, T0e, n):
		'''
			功能：本函数用于求取n自由度机械臂逆运动学方程
			输入参数:DH = [q_init,alpha,a,d];
							qr是参考初值,相对于DH_q0 的转动角度,单位:弧度;
							T0e为DH坐标系确定的DH{0}坐标系与DH{6}之间的关系(目标矩阵);
							n为自由度数,此处为n=7;
			输出参数:qq为相对与DH_q0的转动角度,单位:弧度;已处理到[-pi, pi] 之间
		'''
		efs = 10^(-10)  #误差阀值
		LimitFlag = 0  #标签
		deltaQ = 1   #关节初时刻误差
		E = np.eye(4)  #单位矩阵
		DH_q0 = self.DH0[:,0] #零位关节角
		
		#建立初时刻迭代初值
		q = qr + DH_q0
		alpha = self.DH0[:,1]
		a = self.DH0[:,2]
		d = self.DH0[:,3]
		
		lmax = 0
		l_limit = 1000
		while ((deltaQ > efs) & (LimitFlag == 0)):
			An = E
			for i in range(n):
				T[i] = trans(theta[i],alpha[i],a[i],d[i]),
				An = np.dot(An,T[i])
				A_0_i[i] = An
			
			#计算末端误差
			dA = np.array([[T_h[0:2,3] - An[0:2,3]],[0.5*(np.cross(An[0:2,0],T_h[0:2,0]) + np.cross(An[0:2,1],T_h[0:2,1]) + np.cross(An[0:2,2],T_h[0:2,2 ))]])
										
			#计算雅克比矩阵
			U = E
			Jn = []
			for j in (n - 1 - range(n)):
				U = np.dot(T[j],U)
				
				dd = np.array([[ - U[0,0]*U[1,3] + U[1,0]*U[0,3]],
											[ - U[0,1]*U[1,3] + U[1,1]*U[0,3]],
											[ - U[0,2]*U[1,3] + U[1,2]*U[0,3]]])
				delta = np.transpose(U[2,0:2])
				Jn = [ [ dd; delta],Jn]
			
			#求取关节角
			R = An[0:2,0:2]
			J0 =np.dot( [[R np.zeros(3) ; np.zeros(3)  R]] ,Jn)
			dq = pinv(J0)*dA
			q = q + dq
			deltaQ = np.norm(dq)
			lmax = lmax + 1;
			if (lmax > l_limit):
				LimitFlag = 1
				qq_tmp = q -DH_q0
				
				qq = q_choose(qq_tmp)
				print(" Solution wouldn't converge")

			while (q[i] < - 2*pi):
				q[i] = q[i] +2*pi
		return q

	def position_impedance():
		'''
			功能：建立基于位置阻抗控制



		'''

	def get_impedance_param():
		'''
			功能：获取阻抗参数
		'''
		
	def get_force_feedback():
		'''
			功能：通过力传感器获取末端六维力
		'''

	def get_state_feedback():
		'''
			功能：通过编码器获取关节状态
		'''