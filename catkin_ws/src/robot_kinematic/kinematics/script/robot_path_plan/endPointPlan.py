#-*-coding:utf-8-*-
#本文档为7自由度机械臂末端轨迹生成
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.2.27 2.28
import matplotlib.pyplot as plt
import numpy as np
import FileOpen
import MyPlot
#建立图面坐标系
l = 0.100 #正五角星边长设为100毫米
delta_t = l/400 #每步前进长度

#求取尖点坐标点
y_a = 0
z_a = l*np.cos(18*(3.14/180))

y_b = l*np.sin(18*(3.14/180))
z_b = 0

y_c = -l/2
z_c = l*np.sin(36*(3.14/180))

y_d = l/2
z_d = l*np.sin(36*(3.14/180))

y_e = -l*np.sin(18*(3.14/180))
z_e = 0

#建立末端物体坐标系
x_p = np.zeros(2000)
y_p = np.zeros(2000)
z_p = np.zeros(2000)

#规划AB直线
t = 0
for i in range(400):
	y_p[i] = y_a + t*(y_b - y_a)/l
	z_p[i] = z_a + t*(z_b - z_a)/l
	t = t + delta_t

#规划BC直线
t = 0
for i in range(400,800):
	y_p[i] = y_b + t*(y_c - y_b)/l
	z_p[i] = z_b + t*(z_c - z_b)/l
	t = t + delta_t
	
#规划CD直线
t = 0
for i in range(800,1200):
	y_p[i] = y_c + t*(y_d - y_c)/l
	z_p[i] = z_c + t*(z_d - z_c)/l
	t = t + delta_t

#规划DE直线
t = 0
for i in range(1200,1600):
	y_p[i] = y_d + t*(y_e - y_d)/l
	z_p[i] = z_d + t*(z_e - z_d)/l
	t = t + delta_t

#规划EA直线
t = 0
for i in range(1600,2000):
	y_p[i] = y_e + t*(y_a - y_e)/l
	z_p[i] = z_e + t*(z_a - z_e)/l
	t = t + delta_t
	
#物体坐标系到机械臂0坐标系齐次变化矩阵
x_op = 0.400  #物体坐标原点在固定坐标系的值
y_op = 0
z_op = 0.600
Tp_0= np.eye(4) #物体坐标到机械臂0坐标系齐次变化矩阵
Tp_0[0,3] = x_op #两坐标系方向相同
Tp_0[1,3] = y_op #物体坐标系固定不动
Tp_0[2,3] = z_op

#0坐标系下轨迹表示
xyz_p0 = np.zeros([2000,3])
for i in range(2000):
	P = np.transpose([x_p[i],y_p[i],z_p[i],1])
	P0 = np.dot(Tp_0,P)
	xyz_p0[i,:] = np.transpose(P0[0:3])

#末端工具坐标系姿态保持不变，原点与轨迹重合
n = np.array([0,0,1]) #末端姿态向量
o = np.array([0,-1,0])
a = np.array([1,0,0])

#12参数位置轨迹
pos = np.zeros([2000,12])
for i in range(2000):
	for j in range(12):
		if j < 3:
			pos[i,j] = n[j]
		elif j < 6:
			pos[i,j] = o[j-3]
		elif j < 9:
			pos[i,j] = a[j-6]
		else:
			pos[i,j] = xyz_p0[i,j-9]

#机械臂末端从初始状态到A点
x_oa = x_op #A点在0坐标系下的表示
y_oa = y_op
z_oa = z_op + z_a

x_oend = 0 #机械臂末端在0坐标系下的位置
y_oend = 0
z_oend = 1.125

s = ((x_oend - x_oa)**2 + (y_oend - y_oa)**2 + (y_oend - y_oa)**2 )**0.5 #路径长度
t = 0
delta_t = s/1000
xyz_sf = np.zeros([1000,3])
for i in range(1000):
	xyz_sf[i,0] = x_oend + t*(x_oa - x_oend)/s
	xyz_sf[i,1] = y_oend + t*(y_oa - y_oend)/s
	xyz_sf[i,2] = z_oend + t*(z_oa - z_oend)/s
	t = t + delta_t

#初始位置姿态角
n_end = np.array([-1,0,0]) #末端姿态向量
o_end = np.array([0,-1,0])
a_end = np.array([0,0,1])

#初始位置姿态角到工作状态姿态角
n_s = np.zeros([1000,3])
o_s = np.zeros([1000,3])
a_s = np.zeros([1000,3])
t = 0
delta_t = 1/1000
for i in range(1000):
	n_s[i,:] = n_end + (t/1000)*(n - n_end)
	o_s[i,:] = o_end + (t/1000)*(o - o_end)
	a_s[i,:] = a_end + (t/1000)*(a - a_end)
	t = t + delta_t
	
#12参数去程位置轨迹
pos_began = np.zeros([1000,12])
for i in range(1000):
	for j in range(12):
		if j < 3:
			pos_began[i,j] = n_s[i,j]
		elif j < 6:
			pos_began[i,j] = o_s[i,j-3]
		elif j < 9:
			pos_began[i,j] = a_s[i,j-6]
		else:
			pos_began[i,j] = xyz_sf[i,j-9]
			
#12参数回程位置轨迹
pos_back = np.zeros([1000,12])
for i in range(1000):
	pos_back[i,:] = pos_began[999-i,:]
	pos[:,11]
	
#总轨迹12参数表示
pos_all = np.zeros([4000,12])

for i in range(4000):
	if i < 1000:
		pos_all[i,:] = pos_began[i,:]
	elif i < 3000:
		pos_all[i,:] = pos[i-1000,:]
	else:
		pos_all[i,:] = pos_back[i - 3000,:]


print pos_all.shape
#绘制图形
MyPlot.plot3d(pos_all[:,9],pos_all[:,10],pos_all[:,11],'pentagram_pos')

#将12参数写入文件
FileOpen.write(pos_all,'data/pentagram_pos.txt')
