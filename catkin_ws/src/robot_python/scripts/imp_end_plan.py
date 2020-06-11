#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于规划阻抗运行轨迹
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：2020.6.9

import os
import numpy as np
import math
from math import pi

#自定义函数
from robot_python import Kinematics as kin
from robot_python import RobotParameter as rp
from robot_python import BaseFunction as bf
from robot_python import FileOpen
from robot_python import MyPlot

#DH参数
DH_0 = rp.DHfa_armc
qq_min = rp.q_min_armc
qq_max = rp.q_max_armc

#初始位置
qq_init = np.array([0, 30, 0, 90, 0, 60, 0])*pi/180.0

#正运动学求取末端位置
X_init = kin.fkine_euler(DH_0, qq_init)
print X_init

#设计轨迹
X_end = X_init + np.array([0.15, 0, 0, 0, 0, 0])

#平滑轨迹
num = 1000
t = np.linspace(0, 1, num)
XX = np.zeros([num, 6])
for i in range(num):
    XX[i, :] = X_init + (X_end - X_init)*np.sin(pi/2*t[i])

#笛卡尔单变量随时间绘图
for i in range(6):
    i_string = "Xe " + str(i+1) + "plot"
    MyPlot.plot2d(t, XX[:, i], i_string)

#末端轨迹三维图
xx = np.around(XX[:, 0], decimals=6)
yy = np.around(XX[:, 1], decimals=6)
zz = np.around(XX[:, 2], decimals=6)
MyPlot.plot3d(xx, yy, zz, "3D plot")

#写入文件
parent_path = os.path.abspath('..')
file_name1 = "data/actuator_position.txt"
path = os.path.join(parent_path, file_name1)
FileOpen.write(XX, path)

#求取逆运动学
qq_iter = np.copy(qq_init)
qq = np.zeros([num, 7])
Te = np.eye(4)
for i in range(num):
    Te[0:3, 0:3] = bf.euler_zyx2rot(XX[i, 3:6])
    Te[0:3, 3] = XX[i, 0:3]
    #[qq[i, :], flag] = kin.arm_angle_ikine(Te, 0.0, qq_iter, DH_0, qq_min, qq_max)
    qq[i, :] = kin.iterate_ikine(DH_0, qq_iter, Te, efs=pow(10, -12), i_max=1000)
    qq_iter = qq[i, :]

#绘制关节图
#关节空间单变量随时间绘图
for i in range(7):
    i_string = "qq " + str(i+1) + "plot"
    MyPlot.plot2d(t, qq[:, i], i_string)

#写入文件
file_name2 = "data/joint_position.txt"
path = os.path.join(parent_path, file_name2)
FileOpen.write(qq, path)



