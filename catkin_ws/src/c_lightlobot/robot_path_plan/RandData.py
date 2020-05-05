#-*-coding:utf-8-*-
#本文档为7自由度机械臂工作空间搜索
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.2.23

import numpy as np
import ForwardKinematics as fk
import FileOpen
import MyPlot

#七自由度机械臂D_H参数

a = np.zeros(7)
alpha = [90,90,90,90,90,90,0]
alpha = np.array(alpha)*(np.pi/180)
d = np.array([323.5,0,316,0,284.5,0,201])*0.001
thetaMax = np.array([180,90,180,90,180,90,180])*(np.pi/180)
thetaMin = np.array([-180,-90,-180,-90,-180,-90,-180])*(np.pi/180)

theta = np.zeros([500,7])
pos = np.zeros([500,12])

for i in range(500):
	for j in range(7):
		theta[i,j] = thetaMin[j] + (thetaMax[j] - thetaMin[j])*np.random.random()
	T = fk.T_7to0(theta[i,:],alpha,a,d)
	


	#末端位置用12个参数表示，分别为姿态角旋转矩阵9参数，末端位置3参数
	#写入方式为齐次矩阵前三行从左到右，从上到下
	k = 0
	for j_i in range(4):
		for j_j in range(3):
			pos[i,k] = T[j_j,j_i]
			k = k+1
					
#存储关节角位置
print theta
#FileOpen.write(theta,'data/randtheta500_data1.txt')

#存储位置信息	
print pos
FileOpen.write(pos,'data/randpos500_data2.txt')

#绘制点云图
#MyPlot.plot3Point(pos[:,9],pos[:,10],pos[:,11])

