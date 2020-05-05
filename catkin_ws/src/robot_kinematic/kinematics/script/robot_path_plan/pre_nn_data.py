#-*-coding:utf-8-*-
#本文档用于将关节变量映射到末端空间位置
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.2.25

import MyPlot
import numpy as np
import ForwardKinematics as fk
import FileOpen

#七自由度机械臂D_H参数
theta0 = np.array([0,180,180,180,180,180,0])*(np.pi/180)#初始状态
a = np.zeros(7)
alpha = np.array([90,90,90,90,90,90,0])*(np.pi/180)
d = np.array([323.5,0,316,0,284.5,0,201])*0.001

#初始关节角
theta_k = np.zeros(7)

#读取目标位置参数
data1 = FileOpen.read('data/randtheta_data.txt')
data2 = FileOpen.read('data/wishCircle_theta1.txt')
data3 = FileOpen.read('data/randpos_data.txt')
data3 = np.array(data3)

k = 0
data1 = np.array(data1)
data2 = np.array(data2)

[N,l] = data1.shape
Theta1 = np.zeros([N,l])
#Theta2 = np.zeros([N,l])

x1 = np.zeros(N)
y1 = np.zeros(N)
z1 = np.zeros(N)
x2 = np.zeros(N)
y2 = np.zeros(N)
z2 = np.zeros(N)

for i in range(N):
	for j in range(l):
		Theta1[i,j] = eval(data1[i,j])
		#Theta2[i,j] = eval(data2[i,j])
		

for i in range(N):
	T1 = fk.T_7to0(Theta1[i,:],alpha,a,d)
	#T2 = fk.T_7to0(Theta2[i,:],alpha,a,d)
	x1[k] = T1[0,3]
	y1[k] = T1[1,3]
	z1[k] = T1[2,3]
	x2[k] = data3[i,9]
	y2[k] = data3[i,10]
	z2[k] = data3[i,11]
	k= k+1
	
	'''
	with open('data/PrePos_line3.txt', 'a') as file_to_write:
		for col in range(4):
			for row in range(3):
				file_to_write.write(str(T1[row,col]))
				file_to_write.write(' ')
		file_to_write.write('\n')
'''
data3 = FileOpen.read('data/pentagram_pos.txt')
data3 = np.array(data3)

print data3.shape
print x1.shape

MyPlot.plot3d(x1*1000,y1*1000,z1*1000,'genetics_line','x/mm','y/mm','z/mm')
MyPlot.plot3d(x2*1000,y2*1000,z2*1000,'wish_line','x/mm','y/mm','z/mm')		
MyPlot.plot3Point(x2*1000,y2*1000,z2*1000,x1*1000,y1*1000,z1*1000,'genetic_wish_Circle','x/mm','y/mm','z/mm')		



MyPlot.plot2r(x2*1000,x1*1000,'x_genetic_wish_line','k/point','value/mm',)		
MyPlot.plot2r(y2*1000,y1*1000,'y_genetic_wish_line','k/point','value/mm',)		
MyPlot.plot2r(z2*1000,z1*1000,'z_genetic_wish_line','k/point','value/mm',)		


