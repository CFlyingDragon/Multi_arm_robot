#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于多机械臂规划程序
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020/4/16
import numpy as np
import os
import BaseFunction as bf
import Robots
import RobotParameter as rp
#自定义函数
import FileOpen
import MyPlot
import PathPlan
import Kinematics as kin

#=================直线规划=================#
def line_plan():
    #建立规划类
    linePlan = PathPlan.LinePlan()

    # 获取机器人参数
    DH_0 = rp.DHf_armt
    qq_max = rp.q_max_armc
    qq_min = rp.q_min_armc
    linePlan.get_robot_parameter(DH_0, qq_max, qq_min)

    #规划起点和终点
    Xb = np.array([0.43, 0, 0.013, 0, 3.14, 0])
    Xe = Xb + np.array([0.15, 0, 0, 0, 0, 0])
    linePlan.get_begin_end_point(Xb, Xe)

    #获取起点关节角猜测值
    qq_guess = np.array([0, 60, 0, 60, 0, 60, 0])*np.pi/180.0
    linePlan.get_init_guess_joint(qq_guess)

    #获取姿态角
    R = np.array([[0, 0, 0],
                  [0, 0, 0],
                  [0, 0, 0]])
    #时间和周期
    T = 0.01
    linePlan.get_period(T)
    linePlan.get_plan_time(30)

    #求取关节角度
    qq = linePlan.out_joint()

    #绘制关节角
    num = len(qq)

    t = np.linspace(0, T * (num - 1), num)
    MyPlot.plot2_nd(t, qq, title="qq", lable='qq')

    # 写入文件
    parent_path = os.path.join(os.getcwd(), '../..', 'data/impedance')
    parent_path = os.path.abspath(parent_path)
    file_name = "armt_line_position.txt"
    path = os.path.join(parent_path, file_name)
    FileOpen.write(qq, path)
    return qq

#=================单点力规划=================#
def force_plan():
    # 获取机器人参数
    DH_0 = rp.DHf_armt
    qq_max = rp.q_max_armc
    qq_min = rp.q_min_armc
    kin1 = kin.GeneralKinematic(DH_0)

    #规划起点和终点
    Xb = np.array([0.43, 0, 0.013, 0, 3.14, 0])
    Tb = np.eye(4)
    Tb[0:3, 3] = Xb[0:3]
    Tb[0:3, 0:3] = bf.euler_zyx2rot(Xb[3:6])

    #获取起点关节角猜测值
    qq_guess = np.array([0, 60, 0, 60, 0, 60, 0])*np.pi/180.0

    #获取关节角

    qd = kin1.iterate_ikine(qq_guess, Tb)
    print "qd:", np.round(qd*180/np.pi, 2)

    #时间和周期
    T = 0.01

    #期望力规划：在工具坐标中规划
    num = 1000
    t = np.linspace(0, T*(num-1), num)

    Fd = np.zeros([num, 6])

    #z轴力变换
    fz_c = -10
    f = -10
    Fd[:, 2] = fz_c + f*np.cos(np.pi/2.5*t - np.pi)

    #求取关节角度
    qq = np.zeros([num, 7])
    for i in range(num):
        qq[i, :] = qd

    #绘制关节角
    MyPlot.plot2_nd(t, Fd, title="Fd", lable='fd')
    MyPlot.plot2_nd(t, qq, title="qd", lable='qd')

    # 写入文件
    parent_path = os.path.join(os.getcwd(), '../..', 'data/impedance')
    parent_path = os.path.abspath(parent_path)
    pos_path = parent_path + "/armt_sigle_position1.txt"
    force_path = parent_path + "/armt_sigle_force1.txt"

    FileOpen.write(qq, pos_path)
    FileOpen.write(Fd, force_path)
    return qq

#=================直线规划=================#
def line_force_plan():
    #建立规划类
    linePlan = PathPlan.LinePlan()

    # 获取机器人参数
    DH_0 = rp.DHf_armt
    qq_max = rp.q_max_armc
    qq_min = rp.q_min_armc
    linePlan.get_robot_parameter(DH_0, qq_max, qq_min)

    #规划起点和终点
    Xb = np.array([0.43, 0, 0.013, 0, 3.14, 0])
    Xe = Xb + np.array([0.15, 0, 0, 0, 0, 0])
    linePlan.get_begin_end_point(Xb, Xe)

    #获取起点关节角猜测值
    qq_guess = np.array([0, 60, 0, 60, 0, 60, 0])*np.pi/180.0
    linePlan.get_init_guess_joint(qq_guess)

    #时间和周期
    T = 0.01
    linePlan.get_period(T)
    linePlan.get_plan_time(40)

    #求取关节角度
    qq = linePlan.out_joint()

    #合成位置曲线
    num_init = 500
    num_pos = len(qq)
    num = num_init + num_pos
    t = np.linspace(0, T * (num - 1), num)

    qd = np.zeros([num, 7])
    for i in range(num_init):
        qd[i, :] = qq[0, :]
    qd[num_init:] = qq

    #生成受力曲线
    fd = -10
    Fd = np.zeros([num, 6])
    Fd[:num_init, 2] = fd*np.sin(np.pi/10*t[:num_init])
    for i in range(num_pos):
        Fd[num_init + i, 2] = -10

    # 绘制关节角
    MyPlot.plot2_nd(t, Fd, title="Fd", lable='fd')
    MyPlot.plot2_nd(t, qd, title="qd", lable='qd')

    # 写入文件
    parent_path = os.path.join(os.getcwd(), '../..', 'data/impedance')
    parent_path = os.path.abspath(parent_path)
    pos_path = parent_path + "/armt_line_position2.txt"
    force_path = parent_path + "/armt_line_force2.txt"

    FileOpen.write(qd, pos_path)
    FileOpen.write(Fd, force_path)
    return qq


def main():
    #直线规划
    #line_plan()
    #force_plan()
    line_force_plan()
    print "finish!"


if __name__ == '__main__':
    main()
