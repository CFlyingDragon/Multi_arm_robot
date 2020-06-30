#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于ur逆解
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳）
#时间2020年6月27号

import numpy as np
import math
from math import pi

#********************DH坐标**********************#
DH_0 = np.array([[0.00, -pi/2, 0.0000, 0.1625],
                 [0.00, 0.000, 0.4250, 0.0000],
                 [0.00, 0.000, 0.3922, 0.0000],
                 [0.00, pi/2, 0.0000, 0.1333],
                 [pi/2, pi/2, 0.0000, 0.0997],
                 [0.00, 0.00, 0.0000, 0.0996]])

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

#********************正运动学**********************#
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

#********************逆运动学**********************#
#建立解ms+nc=d的求解函数
def msncd(m,n,d):
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
	#求解T0_1的逆
	T0_1_inv = np.array([[np.cos(qq1), np.sin(qq1), 0, 0],
						 [  		0, 			 0, -1, d1],
						 [-np.sin(qq1), np.cos(qq1 ),  0, 0],
						 [			0 , 			 0,  0, 1]])

	T5_6_inv = np.array([[np.cos(qq6), np.sin(qq6), 0, 0],
						 [-np.sin(qq6), np.cos(qq6), 0, 0],
						 [			  0,  		   0, 1, -d6],
						 [				0, 			 0, 0, 1]])

	T4_5_inv = np.array([[np.cos(qq5), np.sin(qq5),  0, 0],
						 [			0, 			0,  1, -d5],
						 [np.sin(qq5), -np.cos(qq5), 0, 0],
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

	s2_2 = ((a3 * math.cos(qq3_1) + a2) * A[1, 3] - a3 * math.sin(qq3_1) * A[0, 3]) / \
		   (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * math.cos(qq3_1))
	c2_2 = (A[0, 3] + a3 * math.sin(qq3_1) * s2_2) / (a3 * math.cos(qq3_1) + a2)

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
	m1 = d6*Te[0,2] - Te[0,3]
	n1 = Te[1,3] - d6*Te[1,2]
	#求取关节角,第二个数值代表关节角1对应两组解的编号
	[qq1_1,qq1_2] = msncd(m1, n1, d4)

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
	m6_1 = np.sin(qq1_1) * Te[0, 1] - np.cos(qq1_1) * Te[1, 1]
	n6_1 = -np.sin(qq1_1) * Te[0, 0] + np.cos(qq1_1) * Te[1, 0]
	m6_2 = np.sin(qq1_2) * Te[0, 1] - np.cos(qq1_2) * Te[1, 1]
	n6_2 = -np.sin(qq1_2) * Te[0, 0] + np.cos(qq1_2) * Te[1, 0]
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

	print "qq:\n",np.around(qq*180/pi,3)
	#将求解范围转换到[-pi,pi]
	Q = np.zeros([8,6])
	for i in range(8):
		qq[i, :] = qq[i,:] - DH_0[:, 0]
		Q[i, :] =qq_choose(qq[i, :])
	return Q

def main():
    #已知关节角
    qr = np.array([16, -124, 63, 152, 88, -166])*pi/180

    #求取正运动学
    Te = fkine(qr + DH_0[:, 0], DH_0[:, 1], DH_0[:, 2], DH_0[:, 3])
    print "Te:\n", np.around(Te, 3)

    #求取8组逆运动学
    Q = ur_ikine(DH_0, Te)
    print "Q:\n", np.around(Q*180/pi, 3)

if __name__ == "__main__":
    main()