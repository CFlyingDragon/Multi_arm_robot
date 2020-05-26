#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于写示教函数，后期可能会加入一定的学习
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2019.5.20
import numpy as np
import math
from math import pi
import os
import time
import matplotlib.pyplot as plt

#自定义函数模块
from robot_python import Kinematics as kin
from robot_python import PathPlan as pap
from robot_python import MyPlot
from robot_python import EndPoint as enp
from robot_python import RobotParameter as rp
from robot_python import TeachingLearning as tl
from robot_python import FileOpen

#*****示教阶段******#
DH_0 = rp.DH0_armc
qq_path = "/home/d/catkin_ws/src/robot_python/data/teaching/teaching_data.txt"
qq_demo = FileOpen.read(qq_path)
#获取起始点
X0 = kin.fkine_euler(DH_0, qq_demo[0, :])
#获取目标点
X_goal = kin.fkine_euler(DH_0, qq_demo[0, :])
#时间系列
T = 0.01
num = len(qq_demo[:, ])
tt = np.linspace(0, T*(num - 1), num)

#*****学习阶段******#
#末端学习自由度
n_dof = 3

#文件路径
file_path = "/home/d/catkin_ws/src/robot_python/paramter/teaching_learn_param"
dmps_path = file_path + "/dmps_param.txt"
rbf_path = file_path + "/rbf_param.txt"

#创建示教学习器
teach_learn1 = tl.TeachingLearn(n_dof, dmps_path, rbf_path)

#获取机器人参数
teach_learn1.get_robot_paramter(DH_0)

#获取dmps参数
tau = tt[-1] #时间尺度,所有自由度共用
k = np.array([1000, 1000, 1000]) #刚度项
d = np.array([100, 100, 100]) #阻尼项
teach_learn1.get_dmps_paramter(tau, k, d)

#获取rbf参数
h = 1000 #隐藏层个数
alpha = 1 #正则项常数
teach_learn1.get_rbf_paramter(alpha, h)

#获取示教数据
teach_learn1.get_teaching_data(qq_demo, X_goal, X0, T)

#采用最小二乘学习获取权重
teach_learn1.learn()

#将权重写入文件
teach_learn1.write_data()

#获取强迫项
f_demo = teach_learn1.f_demo
for i in range(n_dof):
    i_string = "f_demo" + str(i + 1) + "plot"
    MyPlot.plot2d(tt, f_demo[:, i], i_string)

#*****示教再现阶段******#
#穿件示教再现器
teach_repro1 = tl.TeachingReproduction(dmps_path, rbf_path)

#规划新轨迹
xx0 = X0 + np.zeros([]) #轨迹起点
gg = X_goal #规划目标点
X_data = teach_repro1.reproduction(xx0, gg, tt, T)

#获取强迫项
f = teach_repro1.f
for i in range(n_dof):
    i_string = "f" + str(i + 1) + "plot"
    MyPlot.plot2d(tt, f[:, i], i_string)
print X_data

for i in range(n_dof):
    i_string = "Xe" + str(i + 1) + "plot"
    MyPlot.plot2d(tt, X_data[:, i], i_string)