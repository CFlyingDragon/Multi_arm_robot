#-*-coding:utf-8-*-
#!/usr/bin/env python
#本文档用于画图相关测试
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.12.6
import numpy as np
from math import pi
import os

#自定义函数模块
from robot_python import Kinematics as kin
from robot_python import PathPlan as pap
from robot_python import MyPlot
from robot_python import EndPoint as enp
from robot_python import RobotParameter as rp
from robot_python import FileOpen
from robot_python import JointPlan as jp
from robot_python import BaseFunction as bf

#机械臂的armc的DH参数表
DH_0 = np.copy(rp.DH0_armc)
theta0 = DH_0[:,0]; alpha = DH_0[:,1];a = DH_0[:,2];d = DH_0[:,3]

#U函数测试
# q_max = pi/2
# q_min = -pi/2
# qq = np.linspace(-pi,pi,1000,endpoint=True)
# f = np.zeros(1000)
# for i in range(1000):
#     f[i] = bf.U_function(qq[i],q_max,q_min)
#
# string = 'The U function test!'
# MyPlot.plot2d(qq, f, string)
#
# print bf.U_function(0,q_max,q_min)

def joint_plot():

    path1 = "/home/d/catkin_ws/src/robot_bag/sigle_joint_test/joint_state_position203.txt"
    path2 = "/home/d/catkin_ws/src/robot_bag/sigle_joint_test/joint_state_velocity203.txt"
    joint_position = np.array(FileOpen.read(path1))
    joint_velocity = np.array(FileOpen.read(path2))
    T = 0.02
    kk1 = len(joint_position[:,1])
    kk2 = len(joint_velocity[:,1])
    qq = joint_position[:,1]
    qv = joint_velocity[:,1]
    tt1 = np.linspace(0,(kk1-1)*T,kk1)
    tt2 = np.linspace(0, (kk2 - 1) * T, kk2)
    string1 = 'Joint2 position plot!'
    MyPlot.plot2d(tt1, qq, string1)
    string2 = 'Joint2 velocity plot!'
    MyPlot.plot2d(tt2, qv, string2)

if __name__ == '__main__':
    joint_plot()

