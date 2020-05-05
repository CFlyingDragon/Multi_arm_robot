#-*-coding:utf-8-*-
#本文档用于读取Matlab的数据，并写成新的文件
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.2.22

import matplotlib.pyplot as plt
import numpy as np
import ForwardKinematics as fk
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D

#七自由度机械臂D_H参数
#关节角单位rad,长度单位m
theta0 = np.zeros(7)
a = np.zeros(7)
alpha = [90,90,90,90,90,90,0]
alpha = np.array(alpha)*(np.pi/180)
d = np.array([323.5,0,316,0,284.5,0,201])*0.001

#读取数据	函数
def read(path):
	'''
		本函数用于读取Matlab输出的数据input
		输入参数为文件地址及名称
		末端位置用12个参数表示，分别为姿态角旋转矩阵9参数，末端位置3参数
		写入方式为齐次矩阵前三行从左到右，从上到下
	'''
	data = []
	with open(path, 'r') as file_to_read:
		while True:
			line = file_to_read.readline() # 整行读取数据
			if not line:
				break
			line = line.lstrip()  #去除左边的空格，即去除开头空格
			line = line.rstrip(' \r\n')   #去除末尾空格及转行符
			line=line.split("   ")	 #采用3个空格拆分数据
			data.append(line)
			
	data = np.array(data)   #将数据转为ndarray格式
	[row,col] = data.shape
	print data.shape
	
	#个别数据含有空格，采用循环去除空格
	new_data = np.zeros([row,col])
	for i in range(row):
		for j in range(col):
			new_data[i,j] = data[i,j]
	
	return new_data


#读取目标位置参数
Theta = read('data/Rmp_line_circle.txt')
	
#将单位转换为弧度
Theta = Theta*(3.14/180)

#读取数据形状
[n,l] = Theta.shape
N = np.int(np.floor(n/10))  #采样数据,可修改采样组

print N
x = np.zeros(N)
y = np.zeros(N)
z = np.zeros(N)
m = 0
for i in range(N):
	i = i*10  #与采样组匹配修改
	
	#存储关节角位置
	with open('data/wishCircle_theta1.txt', 'a') as output_to_write:
		for k in range(l):
			theta_data = Theta[i,k]
			#theta_data = round(Theta[i,k],6)  #保存6位精度
			output_to_write.write(str(theta_data))
			output_to_write.write(' ')
		output_to_write.write('\n')
	
	#调用正运动学方程，求解7关节到0关节传递矩阵		
	T0 = fk.T_7to0(Theta[i,:],alpha,a,d)  
	x[m] = T0[0,3]  #提取末端位置
	y[m] = T0[1,3]
	z[m] = T0[2,3]
	m = m+1
		
	#存储位置数据
	#末端位置用12个参数表示，分别为姿态角旋转矩阵9参数，末端位置3参数
	#写入方式为齐次矩阵前三行从左到右，从上到下
	with open('data/wishCircle_line1.txt', 'a') as input_to_write:
		for j_i in range(4):
			for j_j in range(3):
				T_data = T0[j_j,j_i]
				#T_data = round(T_data,6)
				input_to_write.write(str(T_data))
				input_to_write.write(' ')
		input_to_write.write('\n')

#绘制末端位置三维图
mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x, y, z, label='parametric curve')
ax.legend()
plt.show()
