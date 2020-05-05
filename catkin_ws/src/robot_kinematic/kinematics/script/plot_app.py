#-*-coding:utf-8-*-
#!/usr/bin/env python
#本文档用于画图相关测试
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.9.29
import numpy as np
import math
from math import pi
import os

#自定义函数模块
import Kinematics as kin
import PathPlan as pap
import MyPlot
import EndPoint as enp
from RobotParameter import DH0_armc
from RobotParameter import q_limit_armc
import FileOpen
import JointPlan as jp
import BaseFunction as bf

#机械臂的armc的DH参数表
DH_0 = np.copy(DH0_armc)
theta0 = DH0_armc[:,0];alpha = DH0_armc[:,1];a = DH0_armc[:,2];d = DH0_armc[:,3]

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
	l = 0.3
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
	rc = 0.1
	[qr_init,X0_e,T] = enp.circlePoint_armc(rc)
	n = len(qr_init)

	#位置级求逆获取圆的轨迹,3801个点38秒
	[qq3,qv3,qa3] = pap.multipoint_plan_position(qr_init,X0_e,T)
	#[qq3,qv3,qa3] = pap.multipoint_plan_position_aa(qr_init,X0_e,T)
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
		MyPlot.plot2d(tm,m[:,i], i_string)
		
		
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

#=================臂形角参数化求逆模块测试=================#
def analysis_ikine_test():	
	##初始位置
	qr_init = np.array([0,100,0,60,0,30,0])*(pi/180)
	Te = kin.fkine(theta0 + qr_init,alpha,a,d)
	n = len(qr_init)

	##臂形角参数化后求解逆运动学
	psi = pi  
	[q,current_psi,succeed_label] = kin.aa_ikine(Te,psi,qr_init)
	#print np.around(Q*180/pi, decimals=6)
	
	##关节角选泽函数
	#q = kin.analysis_ikine_chose(Q,qr_init)
	print np.around(q*180/pi, decimals=6)
	print current_psi
	print succeed_label
		
	#正运动学求解解
	T0_e = kin.fkine(theta0 + q,alpha,a,d)
	##测试结果：当臂形角等于pi时，关节角发生突变
	###解决办法：臂形角取值范围改为【-pi,pi)
	print T0_e - Te
		
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

#=================零空间运动完整演示模块=================#
def single_joint_test():
	##初始位置
	qr_init = np.array([0,0,0,80,0,0,0])*(pi/180)
	
	n = len(qr_init)
	T = 0.01;
	#从家点运动到初始位置,此处10秒1001个点
	[qq1,qv1,qa1] = jp.home_to_init(qr_init,T)
	k1 = len(qq1[:,0])
	
	#规划完毕，返回家点
	[qq2,qv2,qa2] = jp.go_to_home(qq1[-1,:],T)
	k2 = len(qq2[:,0])

	#合成完整路径
	kk1 = k1; kk2 = k1 + k2;
	k = kk2 
	qq = np.zeros([k,n])
	qq[0:kk1,:] = qq1
	qq[kk1:kk2,:] = qq2
	
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
		i_string = "qq " + str(i+1) + "plot"
		MyPlot.plot2d(t,qq[:,i], i_string)
'''
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
	'''
	
#=================基本函数测试模块=================#
def base_function_test():
	##初始位置
	q_limit = np.copy(q_limit_armc)
	q_test = qr_init = np.array([0,-100,0,60,0,30,0])*(pi/180)
	limit = bf.exceed_joint_limit(q_test,q_limit)
	print limit

def main():
	##末端点规划测试
	#end_point_plan_test()
	
	##直线规划测试
	#line_plan_test()
	
	##多点轨迹跟踪测试
	#multipoint_plan_test()
		
	##圆轨迹演示模块
	circle_plan_demo()
	
	##臂型角参数化求逆模块测试
	#analysis_ikine_test()
	
	##零空间运动规划测试
	#null_space_plan_test()
	
	##零空间全模块演示
	#null_space_plan_demo()
	
	##单关节运动测试
	#single_joint_test()
	
	##基本函数测试测试
	#base_function_test()
	
	print 'finish'

if __name__ == '__main__':
  main()
