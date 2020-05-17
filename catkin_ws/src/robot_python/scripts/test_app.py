#-*-coding:utf-8-*-
#!/usr/bin/env python
#本文档用于画图相关测试
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.9.29
import numpy as np
from math import pi
import os
import time

#自定义函数模块
from robot_python import Kinematics as kin
from robot_python import PathPlan as pap
from robot_python import MyPlot
from robot_python import EndPoint as enp
from robot_python import RobotParameter as rp
from robot_python import FileOpen
from robot_python import JointPlan as jp
from robot_python import BaseFunction as bf
from robot_python import robots_plan

#机械臂的armc的DH参数表
DH_0 = np.copy(rp.DH0_armc)
q_max = np.copy((rp.q_max_armc))
q_min = np.copy((rp.q_min_armc))
theta0 = DH_0[:,0]; alpha = DH_0[:,1];a = DH_0[:,2];d = DH_0[:,3]

#=================末端点笛卡尔空间测试模块=================#
def end_point_plan_test():
	##圆轨迹测试
	rc = 0.1
	[qr_init,X0_e,T] = enp.circlePoint_armc(rc)
	
	print len(X0_e[:,1])
	##直线轨迹测试
	#l = 100
	#[qr_init,X0_e,T] = enp.multipointLine_armc(l)
	#时间变量
	k = len(X0_e[:,0])
	t = np.linspace(0,T*(k-1),k)
	
	#单变量随时间变化绘图
	for i in range(6):
		i_string = "Xe " + str(i+1) + "plot"
		MyPlot.plot2d(t,X0_e[:,i], i_string)
	#末端轨迹三维图
	MyPlot.plot3d(X0_e[:,0],X0_e[:,1],X0_e[:,2],"3D plot")

#=================直线规划轨迹测试模块=================#
def line_plan_test():
	##圆轨迹测试
	#rc = 0.1
	#[qr_init,X0_e,T] = enp.circlePoint_armc(rc)

	##直线轨迹测试
	l = 0.2
	[qr_init,X0_e,T] = enp.multipointLine_armc(l)
	n = len(qr_init)

	#从家点运动到初始位置
	[qq1,qv1,qa1] = jp.home_to_init(qr_init)
	k1 = len(qq1[0,:])

	#停顿点
	qq2 = jp.keep_pos(400,qq1[:,-1])
	k2 = len(qq2[0,:])

	#位置级求逆函数测试测试
	[qq3,qv3,qa3] = pap.multipoint_plan_position(qr_init,X0_e,T)
	# [qq3, qv3, qa3] = pap.multipoint_plan_position_FGA(qr_init, X0_e, T)
	k3 = len(qq3[0,:])

	#停顿点
	qq4 = jp.keep_pos(400,qq3[:,-1])
	k4 = len(qq4[0,:])

	#规划完毕，原路返回
	k = 2*(k1 + k2 + k3) + k4
	qq = np.zeros([n,k])

	#合成完整路径
	kk1 = k1; kk2 = k1 + k2; kk3 = k1 + k2 + k3; kk4 = k1 + k2 + k3 + k4;
	kk5 = k1 + k2 + 2*k3 + k4; kk6 = k1 + 2*k2 + 2*k3 + k4; kk7 = 2*(k1 + k2 + k3) + k4
	qq[:,0:kk1] = qq1
	qq[:,kk1:kk2] = qq2
	qq[:,kk2:kk3] = qq3
	qq[:,kk3:kk4] = qq4
	qq[:,kk4:kk5] = qq3[:,::-1]
	qq[:,kk5:kk6] = qq2[:,::-1]
	qq[:,kk6:kk7] = qq1[:,::-1]

	#关节点写入文档
	parent_path = os.path.abspath('..')
	file_name = "data/position.txt"
	path = os.path.join(parent_path,file_name)
	FileOpen.write(qq.T,path)

	#时间变量
	k = len(qq[0,:]); n = len(qq[:,0])
	t = np.linspace(0,T*(k-1),k)

	#关节空间单变量随时间绘图
	for i in range(n):
		i_string = "qq " + str(i+1) + "plot"
		MyPlot.plot2d(t,qq[i,:], i_string)

	#正运动学求解末端位位置
	Xe = np.zeros([6,k])
	for i in range(k):
		T0_e = kin.fkine(theta0 + qq[:,i],alpha,a,d)
		Xe[0:3,i] = T0_e[0:3,3]
		Xe[3:6,i] = T0_e[0:3,1]

	#笛卡尔单变量随时间绘图
	for i in range(6):
		i_string = "Xe " + str(i+1) + "plot"
		MyPlot.plot2d(t,Xe[i,:], i_string)

	#末端轨迹三维图
	xx = np.around(Xe[0,:], decimals=6)
	yy = np.around(Xe[1,:], decimals=6)
	zz = np.around(Xe[2,:], decimals=6)
	MyPlot.plot3d(xx,yy,zz,"3D plot")

#=================圆规划全轨迹演示模块=================#
def circle_plan_demo():
	##圆轨迹测试
	rc = 0.05
	[qr_init,X0_e,T] = enp.circlePoint_armc(rc)
	n = len(qr_init)

	#位置级求逆获取圆的轨迹,3801个点38秒
	time1 = time.clock()
	#[qq3,qv3,qa3] = pap.multipoint_plan_position(qr_init,X0_e,T)
	[qq3,qv3,qa3] = pap.multipoint_plan_position_aa(qr_init,X0_e,T)
	#[qq3, qv3, qa3,Psi] = pap.multipoint_plan_position_FGA(qr_init, X0_e, T,)
	time2 = time.clock()
	print "plan one time need : %s s" % (time2 - time1)
	#psi_t = np.linspace(0, 38, len(Psi))
	#MyPlot.plot2d(psi_t, Psi, "Psi plot!")
	k3 = len(qq3[:,0])

	#从家点运动到初始位置,此处10秒1001个点
	[qq1,qv1,qa1] = jp.home_to_init(qq3[0,:])
	k1 = len(qq1[:,0])

	#停顿点1，停顿时间4秒，398个点
	qq2 = jp.keep_pos(398,qq1[-1,:])
	k2 = len(qq2[:,0])

	#停顿点2,停顿4秒，398个点
	qq4 = jp.keep_pos(398,qq3[-1,:])
	k4 = len(qq4[:,0])

	#规划完毕，返回家点
	[qq5,qv5,qa5] = jp.go_to_home(qq4[-1,:])
	k5 = len(qq5[:,0])

	#合成完整路径
	kk1 = k1; kk2 = k1 + k2; kk3 = kk2 + k3; kk4 = kk3 + k4; kk5 = kk4 + k5
	k = kk5 
	qq = np.zeros([k,n])
	qq[0:kk1,:] = qq1
	qq[kk1:kk2,:] = qq2
	qq[kk2:kk3,:] = qq3
	qq[kk3:kk4,:] = qq4
	qq[kk4:kk5,:] = qq5

	#关节点写入文档
	parent_path = os.path.abspath('..')
	#file_name = "data/circular_position.txt"
	file_name = "data/circular_position.txt"
	path = os.path.join(parent_path,file_name)
	FileOpen.write(qq,path)

	#时间变量
	k = len(qq[:,0]); n = len(qq[0,:])
	t = np.linspace(0,T*(k-1),k)

	#关节空间单变量随时间绘图
	for i in range(n):
		i_string = "qq " + str(i+1) + "plot"
		MyPlot.plot2d(t,qq[:,i], i_string)

	#正运动学求解末端位位置
	Xe = np.zeros([k,6])
	for i in range(k):
		T0_e = kin.fkine(theta0 + qq[i,:],alpha,a,d)
		Xe[i,0:3] = T0_e[0:3,3]
		Xe[i,3:6] = T0_e[0:3,2]

	#笛卡尔单变量随时间绘图
	for i in range(6):
		i_string = "Xe " + str(i+1) + "plot"
		MyPlot.plot2d(t,Xe[:,i], i_string)

	#末端轨迹三维图
	xx = np.around(Xe[:,0], decimals=6)
	yy = np.around(Xe[:,1], decimals=6)
	zz = np.around(Xe[:,2], decimals=6)
	MyPlot.plot3d(xx,yy,zz,"3D plot")

#=================多点跟踪规划轨迹测试模块=================#
def multipoint_plan_test():
	##圆轨迹测试
	rc = 0.1
	[qr_init,X0_e,T] = enp.circlePoint_armc(rc)

	##直线轨迹测试
	#l = 0.3
	#[qr_init,X0_e,T] = enp.multipointLine_armc(l)
	n = len(qr_init)

	#速度级求逆函数测试测试(规定每行为一个数据点）
	[qq,qv,qa] = pap.multipoint_plan_position(qr_init,X0_e,T)

	#时间变量
	k = len(qq[:,0])
	t = np.linspace(0,T*(k-1)/10.0,k)

	#插值部分测试
	#[s,vel,acc]=pap.spline1(tq,q[:,0],0.01,0,0)
	#i_string = "s " + "plot"
	#MyPlot.plot2d(t,s, i_string)

	#关节空间单变量随时间绘图
	for i in range(n):
		i_string = "qq " + str(i+1) + "plot"
		MyPlot.plot2d(t,qq[:,i], i_string)
		
	#正运动学求解末端位位置
	Xe = np.zeros([k,6])
	for i in range(k):
		T0_e = kin.fkine(theta0 + qq[i,:],alpha,a,d)
		Xe[i,0:3] = T0_e[0:3,3]
		Xe[i,3:6] = T0_e[0:3,2]
	#笛卡尔单变量随时间绘图
	for i in range(6):
		i_string = "Xe " + str(i+1) + "plot"
		MyPlot.plot2d(t,Xe[:,i], i_string)

	#末端轨迹三维图
	xx = np.around(Xe[:,0], decimals=6)
	yy = np.around(Xe[:,1], decimals=6)
	zz = np.around(Xe[:,2], decimals=6)
	MyPlot.plot3d(xx,yy,zz,"3D plot")

#=================三次样条曲线测试模块=================#
def spline_test():
	##生产曲线
	nodeNum = 10
	#x = np.linspace(0,pi,nodeNum)
	t = np.linspace(0,10,nodeNum)
	x = pi/10*t
	y = np.zeros(nodeNum)
	for i in range(nodeNum):
		y[i] = np.sin(x[i])

	#求取加速度测试
	#[m,h] = bf.spline_param(t,y,pi/10,-pi/10)
	##测试结果：正确，但起点和终点速度设置，对加速度影响较大，甚至会出现震荡现象或使其失真，
	##        解决：尽可能给定合适的起始和末端速度

	#单变量加速度随时间绘图
	#i_string = "m " + "plot"
	#MyPlot.plot2d(t,m, i_string)

	#三次样条曲线插值测试
	[s,vel,acc]=bf.spline1(t,y,0.1,0,0)#pi/10,-pi/10)
	##测试结果：正确，但起点和终点速度设置，对加速度影响较大，甚至会出现震荡现象或使其失真，
	##        解决：尽可能给定合适的起始和末端速度

	kk = len(s)
	tt = np.linspace(0,10,kk)
	#单变量位置随时间绘图
	i_string = "s " + "plot"
	MyPlot.plot2d(tt,s, i_string)

	#单变量速度随时间绘图
	i_string = "vel " + "plot"
	MyPlot.plot2d(tt,vel, i_string)

	#单变量加速度随时间绘图
	i_string = "acc " + "plot"
	MyPlot.plot2d(tt,acc, i_string)

	#笛卡尔参数测试
	for i in range(6):
		i_string = "M " + str(i+1) + "plot"
		MyPlot.plot2d(tt,s[:,i], i_string)
		
		
	#正运动学求解末端位位置
	Xe = np.zeros([6,kk])
	for i in range(kk):
		T0_e = kin.fkine(theta0 + s[:,i],alpha,a,d)
		Xe[0:3,i] = T0_e[0:3,3]
		Xe[3:6,i] = T0_e[0:3,1]

	#笛卡尔单变量随时间绘图
	for i in range(6):
		i_string = "Xe " + str(i+1) + "plot"
		MyPlot.plot2d(t,Xe[i,:], i_string)

	#末端轨迹三维图
	xx = np.around(Xe[0,:], decimals=6)
	yy = np.around(Xe[1,:], decimals=6)
	zz = np.around(Xe[2,:], decimals=6)
	MyPlot.plot3d(xx,yy,zz,"3D plot")

#=================臂形角参数化求逆模块测试=================#
def analysis_ikine_test():	
	##初始位置
	qk = np.array([81,20,32,60,91,20,100])*(pi/180)
	qr_init = np.array([80,20,30,60,90,20,100])*(pi/180)
	Te = kin.fkine(theta0 + qr_init,alpha,a,d)

	##臂形角参数化后求解逆运动学
	psi = pi
	tt1 = time.clock()
	[qq, succeed_label] = kin.arm_angle_ikine(Te, psi, qk, DH_0, q_min, q_max)
	tt2 = time.clock()

	print "第一种求解时间：",tt2-tt1

	print"第一种方法：\n", np.around(qq * 180 / pi, decimals=3)

	#正运动学求解解
	T0_e = kin.fkine(theta0 +qq,alpha,a,d)
	print "关节角：\n",np.around(T0_e - Te,6)
		
#=================零空间运动规划测试模块=================#
def null_space_plan_test():	
	##初始位置
	qr_init = np.array([0,30,0,60,0,30,0])*(pi/180)
	Te = kin.fkine(theta0 + qr_init,alpha,a,d)
	n = len(qr_init)

	#获取零空间规划关节空间点
	[qq,qv,qa] = pap.null_space_plan(qr_init,Te)

	#时间变量/规划时间为38秒
	k = len(qq[:,0])
	t = np.linspace(0,38,k)
	
	#MyPlot.plot2d(t,psi, "psi")
	#关节空间单变量随时间绘图
	for i in range(n):
		i_string = "qq " + str(i+1) + "plot"
		MyPlot.plot2d(t,qq[:,i], i_string)
		
	#正运动学求解末端位位置
	Xe = np.zeros([k,6])
	for i in range(k):
		T0_e = kin.fkine(theta0 + qq[i,:],alpha,a,d)
		Xe[i,0:3] = T0_e[0:3,3]
		Xe[i,3:6] = T0_e[0:3,2]
	#笛卡尔单变量随时间绘图
	for i in range(6):
		i_string = "Xe " + str(i+1) + "plot"
		MyPlot.plot2d(t,Xe[:,i], i_string)

	#末端轨迹三维图
	xx = np.around(Xe[:,0], decimals=6)
	yy = np.around(Xe[:,1], decimals=6)
	zz = np.around(Xe[:,2], decimals=6)
	MyPlot.plot3d(xx,yy,zz,"3D plot")

#=================零空间运动完整演示模块=================#
def null_space_plan_demo():
	##初始位置
	qr_init = np.array([0,30,0,30,0,30,0])*(pi/180)
	Te = kin.fkine(theta0 + qr_init,alpha,a,d)
	n = len(qr_init)
	T= 0.1
	
	#获取零空间规划关节空间点
	[qq3,qv3,qa3] = pap.null_space_plan(qr_init,Te)

	#时间变量/规划时间为38秒
	k3 = len(qq3[:,0])

	#从家点运动到初始位置,此处10秒1001个点
	[qq1,qv1,qa1] = jp.home_to_init(qq3[0,:])
	k1 = len(qq1[:,0])

	#停顿点1，停顿时间4秒，398个点
	qq2 = jp.keep_pos(398,qq1[-1,:])
	k2 = len(qq2[:,0])

	#停顿点2,停顿4秒，398个点
	qq4 = jp.keep_pos(398,qq3[-1,:])
	k4 = len(qq4[:,0])

	#规划完毕，返回家点
	[qq5,qv5,qa5] = jp.go_to_home(qq4[-1,:])
	k5 = len(qq5[:,0])

	#合成完整路径
	kk1 = k1; kk2 = k1 + k2; kk3 = kk2 + k3; kk4 = kk3 + k4; kk5 = kk4 + k5
	k = kk5 
	qq = np.zeros([k,n])
	qq[0:kk1,:] = qq1
	qq[kk1:kk2,:] = qq2
	qq[kk2:kk3,:] = qq3
	qq[kk3:kk4,:] = qq4
	qq[kk4:kk5,:] = qq5

	#关节点写入文档
	parent_path = os.path.abspath('..')
	#file_name = "data/null_position.txt"
	file_name = "data/position.txt"
	path = os.path.join(parent_path,file_name)
	FileOpen.write(qq,path)

	#时间变量
	k = len(qq[:,0]); n = len(qq[0,:])
	t = np.linspace(0,T*(k-1),k)

	#关节空间单变量随时间绘图
	for i in range(n):
		i_string = "qq " + str(i+1) + "plot"
		MyPlot.plot2d(t,qq[:,i], i_string)

	#正运动学求解末端位位置
	Xe = np.zeros([k,6])
	for i in range(k):
		T0_e = kin.fkine(theta0 + qq[i,:],alpha,a,d)
		Xe[i,0:3] = T0_e[0:3,3]
		Xe[i,3:6] = T0_e[0:3,2]

	#笛卡尔单变量随时间绘图
	for i in range(6):
		i_string = "Xe " + str(i+1) + "plot"
		MyPlot.plot2d(t,Xe[:,i], i_string)

	#末端轨迹三维图
	xx = np.around(Xe[:,0], decimals=6)
	yy = np.around(Xe[:,1], decimals=6)
	zz = np.around(Xe[:,2], decimals=6)
	MyPlot.plot3d(xx,yy,zz,"3D plot")

#=================单关节测试模块=================#
def single_joint_test():
	##初始位置
	qr_init = np.array([0,80,0,0,0,0,0])*(pi/180)
	tt = 8
	n = len(qr_init)
	T = 0.0025;
	#从家点运动到初始位置,此处10秒1001个点
	[qq1,qv1,qa1] = jp.home_to_init(qr_init,T,tt)
	k1 = len(qq1[:,0])
	
	#规划完毕，返回家点
	[qq2,qv2,qa2] = jp.go_to_home(qq1[-1,:],T,tt)
	k2 = len(qq2[:,0])

	#合成完整路径
	kk1 = k1; kk2 = k1 + k2;
	k = kk2 
	qq = np.zeros([k,n])
	qq[0:kk1,:] = qq1
	qq[kk1:kk2,:] = qq2
	qv = np.zeros([k, n])
	qv[0:kk1, :] = qv1
	qv[kk1:kk2, :] = qv2
	
	#关节点写入文档
	parent_path = os.path.abspath('..')
	file_name = "data/position.txt"
	path = os.path.join(parent_path,file_name)
	FileOpen.write(qq,path)

	#时间变量
	k = len(qq[:,0]); n = len(qq[0,:])
	t = np.linspace(0,T*(k-1),k)

	#关节空间单变量随时间绘图
	for i in range(n):
		i_string = "qq" + str(i+1) + "plot"
		MyPlot.plot2d(t,qq[:,i], i_string)
		i_string = "qv" + str(i + 1) + "plot"
		MyPlot.plot2d(t, qv[:, i], i_string)
		# 关节空间单变量随时间绘图

#=================基本函数测试模块=================#
def base_function_test():
	##初始位置
	q_limit = np.copy(rp.q_limit_armc)
	q_test = qr_init = np.array([0,-100,0,60,0,30,0])*(pi/180)
	limit = bf.exceed_joint_limit(q_test,q_limit)
	print limit

# =================关节空间规划，运动到定点=================#
def joint_plan_test():
	##初始位置
	qr_home = np.array([0, 0, 0, 0, 0, 0]) * (pi / 180)
	qr_init = np.array([-180, -90, 0, -90, 0, 0]) * (pi / 180)
	tt = 8
	n = len(qr_init)
	T = 0.0025;
	# 从家点运动到初始位置,此处10秒1001个点
	[qq1, qv1, qa1] = jp.home_to_init(qr_init, T, tt)
	k1 = len(qq1[:, 0])

	#在初始位置保持
	k2 = 40000
	qq2 = jp.keep_pos(k2, qq1[-1,:])
	# 规划完毕，返回家点
	[qq3, qv3, qa3] = jp.go_to_home(qq1[-1, :], T, tt)
	k3 = len(qq3[:, 0])

	# 合成完整路径
	kk1 = k1;
	kk2 = k1 + k2;
	kk3 = kk2 + k3;
	k = kk3
	qq = np.zeros([k, n])
	qq[0:kk1, :] = qq1
	qq[kk1:kk2, :] = qq2
	qq[kk2:kk3, :] = qq3
	qv = np.zeros([k, n])
	qv[0:kk1, :] = qv1
	qv[kk2:kk3, :] = qv3

	# 关节点写入文档
	parent_path = os.path.abspath('..')
	file_name = "data/ur5_position2.txt"
	path = os.path.join(parent_path, file_name)
	FileOpen.write(qq, path)

	# 时间变量
	k = len(qq[:, 0]);
	n = len(qq[0, :])
	t = np.linspace(0, T * (k - 1), k)

# =================双臂物体搬运测试=================#
def ur5s_plan_test():
	##初始位置
	qr_home = np.array([0, 0, 0, 0, 0, 0]) * (pi / 180)
	qr_init = np.array([-180, -90, 0, -90, 0, 0]) * (pi / 180)
	tt = 8
	n = len(qr_init)
	T = 0.0025;
	# 从家点运动到初始位置,此处10秒1001个点
	[qq1, qv1, qa1] = jp.home_to_init(qr_init, T, tt)
	k1 = len(qq1[:, 0])

	#在初始位置保持
	k2 = 40000
	qq2 = jp.keep_pos(k2, qq1[-1,:])
	# 规划完毕，返回家点
	[qq3, qv3, qa3] = jp.go_to_home(qq1[-1, :], T, tt)
	k3 = len(qq3[:, 0])

	# 合成完整路径
	kk1 = k1;
	kk2 = k1 + k2;
	kk3 = kk2 + k3;
	k = kk3
	qq = np.zeros([k, n])
	qq[0:kk1, :] = qq1
	qq[kk1:kk2, :] = qq2
	qq[kk2:kk3, :] = qq3
	qv = np.zeros([k, n])
	qv[0:kk1, :] = qv1
	qv[kk2:kk3, :] = qv3

	# 关节点写入文档
	parent_path = os.path.abspath('..')
	file_name = "data/ur5_position2.txt"
	path = os.path.join(parent_path, file_name)
	FileOpen.write(qq, path)

	# 时间变量
	k = len(qq[:, 0]);
	n = len(qq[0, :])
	t = np.linspace(0, T * (k - 1), k)

# =================加权最小范数法求逆模块测试=================#
def w_ikine_test():
	##初始位置
	qr = np.array([90, 40, 30, 80, 0, 30, 90]) * (pi / 180)
	qr_init = np.array([91, 41, 31, 81, 1, 31, 91]) * (pi / 180)
	Te = kin.fkine(theta0 + qr_init, alpha, a, d)
	n = len(qr_init)

	##臂形角参数化后求解逆运动学
	d_h0 = np.zeros(n)
	tt1 = time.clock()
	[qq, dh] = kin.w_ikine(DH_0,qr,q_max,q_min,d_h0,Te)
	tt2 = time.clock()
	print "逆运动学时间：",tt2 - tt1
	# print np.around(Q*180/pi, decimals=6)

	##关节角选泽函数
	# q = kin.analysis_ikine_chose(Q,qr_init)
	print np.around(qq * 180 / pi, decimals=6)
	print dh

	# 正运动学求解解
	T0_e = kin.fkine(theta0 + qq, alpha, a, d)
	##测试结果：当臂形角等于pi时，关节角发生突变
	###解决办法：臂形角取值范围改为【-pi,pi)
	print T0_e - Te

# =================雅克比两种求解方法=================#
def jaco_test():
	##初始位置
	DH_0 = rp.DH0_ur5
	qr = np.array([100, 50, 40, 80, 100, 30]) * (pi / 180)

	#采用构造发求雅克比矩阵
	t1 = time.clock()
	Jaco1= kin.jacobian(DH_0, qr)
	t2 = time.clock()

	#采用迭代法求雅克比矩阵
	Jaco2 = kin.jeco_0(DH_0, qr)
	t3 = time.clock()

	#信息显示
	print "构造法求取雅克比：\n",np.around(Jaco1,6)
	print "构造法求取雅克比所需时间：",t2 - t1
	print "迭代法求取雅克比：\n", np.around(Jaco2,6)
	print "迭代法求取雅克比所需时间：", t3 - t2

# =================ur逆解测试=================#
def ur_ikine_test():
	##初始位置
	DH_0 = rp.DH0_ur5
	theta0 = DH_0[:,0]
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	qr = np.array([100, 50, 40, 80, 100, 30]) * (pi / 180)
	qq_k = np.array([99, 52, 43, 81, 100, 31]) * (pi / 180)
	n = 6

	#正运动学求取末端位姿
	tt1 = time.clock()
	Te = kin.fkine(theta0 + qr, alpha, a, d)
	tt2 = time.clock()

	#采用解析解求8组逆解
	Q = kin.ur_ikine(DH_0,Te)
	tt3 = time.clock()
	print np.around(Q*180/pi, decimals=6)
	print "正解所需时间：", tt2 - tt1
	print "逆解所需时间：", tt3 - tt2

	# 正运动学求解解
	for i in range(8):
		qq = Q[i,:]
		T0_e = kin.fkine(theta0 + qq, alpha, a, d)
		print "第",i,"组误差：",np.around(T0_e - Te,7)

	#求取唯一解
	tt4 = time.clock()
	qq = kin.ur_ikine_choice(DH_0,Te,qq_k)
	tt5 = time.clock()
	print np.around(qq* 180 / pi, decimals=6)
	print "求唯一解所选要时间：", tt5 - tt4


def main():
	##末端点规划测试
	#end_point_plan_test()
	
	##直线规划测试
	#line_plan_test()
	
	##多点轨迹跟踪测试
	#multipoint_plan_test()
		
	##圆轨迹演示模块
	#circle_plan_demo()
	
	##臂型角参数化求逆模块测试
	analysis_ikine_test()
	
	##零空间运动规划测试
	#null_space_plan_test()
	
	##零空间全模块演示
	#null_space_plan_demo()
	
	##单关节运动测试
	#single_joint_test()
	
	##基本函数测试测试
	#base_function_test()

	##多臂规划程序
	#arms_line_plan_test()

	##加权最小范数求逆运动学
	#w_ikine_test()

	##定点运动，关节空间规划
	#joint_plan_test()

	##ur解析解测试
	#ur_ikine_test()

	##雅克比测试
	#jaco_test()
	
	print 'finish'

if __name__ == '__main__':
  main()
