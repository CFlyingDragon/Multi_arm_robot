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
from robot_python import robots_plan as rpl

#机械臂的armc的DH参数表
DH_0 = np.copy(rp.DH0_armc)
theta0 = DH_0[:,0]; alpha = DH_0[:,1];a = DH_0[:,2];d = DH_0[:,3]

#=================工件移动轨迹测试模块=================#
def end_point_plan_test():
	#初始值
	T = 0.1
	##末端物体移动轨迹
	Xw = rpl.workpiece_moving_track()
	
	print len(Xw[:,1])
	#时间变量
	k = len(Xw[:,0])
	t = np.linspace(0,T*(k-1),k)
	
	#单变量随时间变化绘图
	for i in range(6):
		i_string = "Xw " + str(i+1) + "plot"
		MyPlot.plot2d(t,Xw[:,i], i_string)
	#末端轨迹三维图
	MyPlot.plot3d(Xw[:,0],Xw[:,1],Xw[:,2],"3D plot")

#=================双臂直线规划轨迹测试模块=================#
def arms_line_plan_test():

	##机器人末端轨迹
	[Xe1,Xe2] = robots_plan.arms_end_track(0.8)
	T = 0.1
	print "Xe1 = %s" % Xe1[0,:]
	print "Xe2 = %s" % Xe2[0, :]

	#从家点运动到初始位置
	q1_ready = np.array([0,pi/6,0,pi/6,0,pi/6,0])
	q2_ready = np.array([0, -pi / 6, 0, -pi / 6, 0, -pi / 6, 0])
	Te1 = np.eye(4)
	Te1[0:3,0:3] = bf.euler_zyx2rot(Xe1[0,3:6])
	Te1[3,0:3] =  Xe1[0,0:3]
	qr1_init = kin.iterate_ikine(DH_0, q1_ready, Te1)

	Te2 = np.eye(4)
	Te2[0:3, 0:3] = bf.euler_zyx2rot(Xe2[0, 3:6])
	Te2[3, 0:3] = Xe2[0, 0:3]
	qr2_init = kin.iterate_ikine(DH_0, q2_ready, Te2)

	# 位置级求逆函数测试测试
	[qq1_2,qv1_2,qa1_2] = pap.multipoint_plan_position(qr1_init, Xe1, T)
	[qq2_2, qv2_2, qa2_2] = pap.multipoint_plan_position(qr2_init, Xe2, T)

	#从家点到初始位置
	[qq1_1, qv1_1, qa1_1] = jp.home_to_init(qq1_2[0,:])
	[qq2_1, qv2_1, qa2_1] = jp.home_to_init(qq2_2[0, :])

	#从初始位置回到家点
	[qq1_3, qv1_3, qa1_3] = jp.go_to_home(qq1_2[-1,:])
	[qq2_3, qv2_3, qa2_3] = jp.go_to_home(qq2_2[-1, :])

	#计算数据长度
	k1_1 = len(qq1_1[:,0]);k2_1 = len(qq2_1[:,0])
	k1_2 = len(qq1_2[:, 0]);k2_2 = len(qq2_2[:, 0])
	k1_3 = len(qq1_3[:, 0]);k2_3 = len(qq2_3[:, 0])

	k = k1_1 + k1_2 + k1_3
	k2 = k2_1 + k2_2
	qq1 = np.zeros([k,7])
	qq2 = np.zeros([k, 7])

	#合成完整路径
	qq1[0:k1_1,:] = qq1_1
	qq1[k1_1:k2, :] = qq1_2
	qq1[k2:k, :] = qq1_3

	qq2[0:k2_1, :] = qq2_1
	qq2[k2_1:k2 , :] = qq2_2
	qq2[k2 :k, :] = qq2_3

	TT = 0.01
	num = len(qq1[:, 0])
	tt = np.linspace(0, TT * (num - 1), num)

	# 笛卡尔单变量随时间绘图
	for i in range(7):
		i_string = "qq1_ " + str(i + 1) + "plot"
		MyPlot.plot2d(tt, qq1[:, i], i_string)
	for i in range(7):
		i_string = "qq2_ " + str(i + 1) + "plot"
		MyPlot.plot2d(tt, qq2[:, i], i_string)

	#关节点写入文档
	parent_path = os.path.abspath('..')
	file_name1 = "data/position1.txt"
	path1 = os.path.join(parent_path,file_name1)
	FileOpen.write(qq1,path1)

	file_name2 = "data/position2.txt"
	path2 = os.path.join(parent_path, file_name2)
	FileOpen.write(qq2, path2)

# =================加权最小范数法求关节角逆解=================#
def w_ikine_test():
	##初始位置
	qr_init = np.array([0, 100, 0, 60, 0, 30, 0]) * (pi / 180)
	Te = kin.fkine(theta0 + qr_init, alpha, a, d)
	n = len(qr_init)
	d_h0 = np.zeros(n)
	q_max = rp.q_max_armc
	q_min = rp.q_min_armc

	##最小求解逆运动学zuixoi
	[qq,dh] = kin.w_ikine(DH_0,qr_init,q_max,q_min,d_h0,Te)
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

	# # 关节空间单变量随时间绘图
	# for i in range(n):
	# 	i_string = "qq" + str(i + 1) + "plot"
	# 	MyPlot.plot2d(t, qq[:, i], i_string)
	# 	i_string = "qv" + str(i + 1) + "plot"
	# 	MyPlot.plot2d(t, qv[:, i], i_string)

def main():
	##末端点规划测试
	end_point_plan_test()

	##多臂规划程序
	#arms_line_plan_test()

	##加权最小范数求逆运动学
	#w_ikine_test()

	##定点运动，关节空间规划
	#joint_plan_test()
	
	print 'finish'

if __name__ == '__main__':
  main()
