#-*-coding:utf-8-*-
#!/usr/bin/env python
#本文档用于7自由度机械臂的关节空间规划
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.10.21
import numpy as np
import math

#自定义函数模块
import Kinematics as kin
import PathPlan as pap
from RobotParameter import DH0_armc as DH_0
import EndPoint as enp
import BaseFunction as bf

#=====================从规划初始位置到家点=====================#
def go_to_home(qq_init,T = 0.01,tf = 4):
	'''
		本函数用于将机械臂返回零点
	'''
	print "return home"
	n = len(qq_init)
	qq_home = np.zeros(n)
	q0 = np.zeros(n)
	dt = T
	[qq,qv,qa] = bf.interp5rdPoly(qq_init,q0,q0,qq_home,q0,q0,tf,dt)
	return [qq,qv,qa]

#=====================从家点到规划初始位置=====================#
def home_to_init(qq_init,T = 0.01,tf = 4):
	'''
		本函数用于将机械臂运动到规划初始点
	'''
	n = len(qq_init)
	qq_home = np.zeros(n)
	q0 = np.zeros(n)
	dt = T
	[qq,qv,qa] = bf.interp5rdPoly(qq_home,q0,q0,qq_init,q0,q0,tf,dt)
	return [qq,qv,qa]

#=====================保持位置=====================#
def keep_pos(k,qq0):
	'''
		本函数用于将机械臂返回零点
	'''
	n = len(qq0)
	qq = np.zeros([k,n])
	for i in range(k):
		qq[i,:] = qq0
	return qq

#=====================从一个起点到另一个点=====================#
def pos1_to_pos2(qq1,qq2,T,tf):
	n = len(qq1)
	q0 = np.zeros(n)
	[qq, qv, qa] = bf.interp5rdPoly(qq1, q0, q0, qq2, q0, q0, tf, T)
	return [qq, qv, qa]