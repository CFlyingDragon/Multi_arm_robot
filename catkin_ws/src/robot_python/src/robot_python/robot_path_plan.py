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
import JointPlan as jp
import matplotlib.pyplot as plt
#解决中文显示问题
plt.rcParams['font.sans-serif'] = ['SimHei'] # 指定默认字体
plt.rcParams['axes.unicode_minus'] = False # 解决保存图像是负号'-'显示为方块的问题

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
def force_plan_armt():
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

#===============刚度测量：给定末端期望力=================#
def stiff_force_plan_armt():
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

#=================刚度测量：给定末端期望力=================#
def force_plan_armc_stiff():
    # 获取机器人参数
    DH_0 = rp.DHfa_armc
    qq_max = rp.q_max_armc
    qq_min = rp.q_min_armc
    kin1 = kin.GeneralKinematic(DH_0)

    #规划起点和终点
    Xb = np.array([0.454, 0, 0.013, 0, 3.14, 0])
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
    num = 6500
    t = np.linspace(0, T*(num-1), num)

    Fd = np.zeros([num, 6])

    #z轴力变换
    for i in range(num):
        if(i>num/2):
            j = num - i
        else:
            j = i
        if(j<1000):
            Fd[i, 2] = -5
        else:
            Fd[i, 2] = (j/500)*(-5)

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
    pos_path = parent_path + "/armc_stiff_position1.txt"
    force_path = parent_path + "/armc_stiff_force1.txt"

    FileOpen.write(qq, pos_path)
    FileOpen.write(Fd, force_path)
    return qq

#=================直线规划=================#
def line_force_plan_armt():
    #建立规划类
    linePlan = PathPlan.LinePlan()

    # 获取机器人参数
    DH_0 = rp.DHf_armt
    qq_max = rp.q_max_armt
    qq_min = rp.q_min_armt
    linePlan.get_robot_parameter(DH_0, qq_min, qq_max)

    #规划起点和终点
    Xb = np.array([0.42, 0, 0.020, 0, 3.14, 0])
    Xe = Xb + np.array([0.14, 0, 0, 0, 0, 0])
    linePlan.get_begin_end_point(Xb, Xe)

    #获取起点关节角猜测值
    qq_guess = np.array([0, 46, 0, 90, 0, 42, 0])*np.pi/180.0
    linePlan.get_init_guess_joint(qq_guess)

    #时间和周期
    T = 0.01
    linePlan.get_period(T)
    linePlan.get_plan_time(40)

    #求取关节角度
    qq = linePlan.out_joint()

    #合成位置曲线
    num_init = 501
    num_pos = len(qq)
    num = num_init + num_pos
    t = np.linspace(0, T * (num - 1), num)

    qd = np.zeros([num, 7])
    for i in range(num_init):
        qd[i, :] = qq[0, :]
    qd[num_init:] = qq

    #生成受力曲线
    fd = -5
    Fd = np.zeros([num, 6])
    [Fd[:num_init, 2], _, _] = bf.interp5rdPoly1(0.0, 0.0, 0.0, fd, 0.0, 0.0, 5, 0.01)
    for i in range(num_pos):
        Fd[num_init + i, 2] = -5

    # 绘制关节角
    MyPlot.plot2_nd(t, Fd, title="Fd", lable='fd')
    MyPlot.plot2_nd(t, qd, title="qd", lable='qd')

    # 写入文件
    parent_path = os.path.join(os.getcwd(), '../..', 'data/impedance')
    parent_path = os.path.abspath(parent_path)
    pos_path = parent_path + "/armt_line_position3.txt"
    force_path = parent_path + "/armt_line_force3.txt"

    FileOpen.write(qd, pos_path)
    FileOpen.write(Fd, force_path)
    return qq

def line_force_plan_armc():
    #建立规划类
    linePlan = PathPlan.LinePlan()

    # 获取机器人参数
    DH_0 = rp.DHfa_armc
    qq_max = rp.q_max_armc
    qq_min = rp.q_min_armc
    kin1 = kin.GeneralKinematic(DH_0, rp.q_min_armc, rp.q_max_armc)
    linePlan.get_robot_parameter(DH_0, qq_min, qq_max)

    #规划起点和终点
    Xb = np.array([0.410, 0, 0.022, 0, 3.14, 0])
    Xe = Xb + np.array([0.15, 0, 0, 0, 0, 0])
    linePlan.get_begin_end_point(Xb, Xe)

    #获取起点关节角猜测值
    qq_guess = np.array([0, 30, 0, 85, 0, 65, 0])*np.pi/180.0
    linePlan.get_init_guess_joint(qq_guess)

    #时间和周期
    T = 0.01
    linePlan.get_period(T)
    linePlan.get_plan_time(50)

    #求取关节角度
    qq = linePlan.out_joint()

    #合成位置曲线
    num_init = 1000
    num_pos = len(qq)
    num = num_init + num_pos
    t = np.linspace(0, T * (num - 1), num)

    qd = np.zeros([num, 7])
    for i in range(num_init):
        qd[i, :] = qq[0, :]
    qd[num_init:] = qq

    #获得末端位姿
    XX_c = np.zeros([num, 3])
    for i in range(num):
        Te = kin1.fkine(qd[i, :])
        XX_c[i, :] = Te[0:3, 3]

    #生成受力曲线
    fd = -5
    Fd = np.zeros([num, 6])
    Fd[:num_init, 2] = fd*np.sin(np.pi/20*t[:num_init])
    for i in range(num_pos):
        Fd[num_init + i, 2] = fd
    #绘制期望力
        # 绘制数据图
    dpi = 500
    # 搬运臂期望位置
    plt.figure(1)
    plt.plot(t, 1000*XX_c[:, 0], linewidth='2', label='x', color='r', linestyle='--')
    plt.plot(t, 1000*XX_c[:, 1], linewidth='2', label='y', color='g', linestyle='-.')
    plt.plot(t, 1000*XX_c[:, 2], linewidth='2', label='z', color='b')
    plt.title(u"曲面恒力跟踪期望位置")
    plt.xlabel("t(s)")
    plt.ylabel("X(mm)")
    plt.ylim(-100, 800)
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    # 曲面直线
    plt.figure(2)
    plt.ylim(-6, 0)
    plt.plot(t, Fd[:, 2], linewidth='2', label='Fz', color='b')
    plt.title(u"曲面力恒跟踪期望力")
    plt.xlabel("t(s)")
    plt.ylabel("Fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    # 绘制关节角
    MyPlot.plot2_nd(t, Fd, title="Fd", lable='fd')
    MyPlot.plot2_nd(t, qd, title="qd", lable='qd')

    # 写入文件
    parent_path = os.path.join(os.getcwd(), '../..', 'data/impedance')
    parent_path = os.path.abspath(parent_path)
    pos_path = parent_path + "/armc_line_position2.txt"
    force_path = parent_path + "/armc_line_force2.txt"

    FileOpen.write(qd, pos_path)
    FileOpen.write(Fd, force_path)
    return qq

#=================重复定位规划=================#
def repeat_positionin_plan():
    #建立规划类
    linePlan = PathPlan.LinePlan()

    # 获取机器人参数
    DH_0 = rp.DH0_armc
    qq_max = rp.q_max_armc
    qq_min = rp.q_min_armc
    linePlan.get_robot_parameter(DH_0, qq_min, qq_max)
    my_kin = kin.GeneralKinematic(DH_0, qq_min, qq_max)

    #建立测试平面
    l = 0.08
    P1 = np.array([0, 0, 0, 1.0])
    P2 = np.array([-l, -l, 0, 1.0])
    P3 = np.array([l, -l, 0, 1.0])
    P4 = np.array([l, l, 0, 1.0])
    P5 = np.array([-l, l, 0, 1.0])

    #测试平面到基坐标系转换
    qq0 = np.array([0, -45, 0, 30, 0, 60, 0.0])*np.pi/180.0
    T0_c = my_kin.fkine(qq0)
    print "tc:", np.around(T0_c, 4)
    P2 = np.dot(T0_c, P2)
    P3 = np.dot(T0_c, P3)
    P4 = np.dot(T0_c, P4)
    P5 = np.dot(T0_c, P5)

    X1 = my_kin.fkine_euler(qq0)
    X2 = np.copy(X1)
    X2[0:3] = P2[0:3]
    X3 = np.copy(X1)
    X3[0:3] = P3[0:3]
    X4 = np.copy(X1)
    X4[0:3] = P4[0:3]
    X5 = np.copy(X1)
    X5[0:3] = P5[0:3]
    print "X1:", np.around(X1, 3)
    print "X2:", np.around(X2, 3)
    print "X3:", np.around(X3, 3)
    print "X4:", np.around(X4, 3)
    print "X5:", np.around(X5, 3)
    # 时间和周期
    T = 0.01
    linePlan.get_period(T)
    linePlan.get_plan_time(8)
    #规划起点和终点
    linePlan.get_begin_end_point(X1, X2)
    linePlan.get_init_guess_joint(qq0)
    qq1 = linePlan.out_joint()

    linePlan.get_begin_end_point(X2, X3)
    linePlan.get_init_guess_joint(qq1[-1, :])
    qq2 = linePlan.out_joint()

    linePlan.get_begin_end_point(X3, X4)
    linePlan.get_init_guess_joint(qq2[-1, :])
    qq3 = linePlan.out_joint()

    linePlan.get_begin_end_point(X4, X5)
    linePlan.get_init_guess_joint(qq3[-1, :])
    qq4 = linePlan.out_joint()

    linePlan.get_begin_end_point(X5, X1)
    linePlan.get_init_guess_joint(qq4[-1, :])
    qq5 = linePlan.out_joint()

    q0 = np.zeros(7)
    [qq6, _, _] = bf.interp5rdPoly(qq5[-1, :], q0, q0, qq1[0, :], q0, q0, 1.0, T)

    #直接关节空间规划
    k1 = len(qq1)
    k2 = len(qq2)
    k3 = len(qq3)
    k4 = len(qq4)
    k5 = len(qq5)
    k6 = len(qq6)

    ks = 200
    n = len(qq0)
    kk = k1 + k2 + k3 + k4 + k5 + 5*ks + k6

    qq = np.zeros([kk, n])
    k = 0

    qq[k:k1, :] = qq1
    k = k + k1
    qq[k:k+ks] = np.dot(np.ones([ks, n]), np.diag(qq1[-1, :]))
    k = k + ks
    qq[k:k + k2, :] = qq2
    k = k + k2
    qq[k:k + ks] = np.dot(np.ones([ks, n]), np.diag(qq2[-1, :]))
    k = k + ks
    qq[k:k + k3, :] = qq3
    k = k + k3
    qq[k:k + ks] = np.dot(np.ones([ks, n]), np.diag(qq3[-1, :]))
    k = k + ks
    qq[k:k + k4, :] = qq4
    k = k + k4
    qq[k:k + ks] = np.dot(np.ones([ks, n]), np.diag(qq4[-1, :]))
    k = k + ks
    qq[k:k + k5, :] = qq5
    k = k + k5
    qq[k:k+k6, :] = qq6
    k = k + k6
    qq[k:k + ks] = np.dot(np.ones([ks, n]), np.diag(qq6[-1, :]))

    t = np.linspace(0, T * (kk - 1), kk)
    MyPlot.plot2_nd(t, qq, title="qq", lable='qq')

    m = 10
    qq_m = np.zeros([m*kk, n])
    for i in range(m):
        qq_m[i*kk:(i+1)*kk, :] = qq

    tm = np.linspace(0, T * (m*kk - 1), m*kk)
    MyPlot.plot2_nd(tm, qq_m, title="qq_m", lable='qq_m')
    # 写入文件
    parent_path = os.path.join(os.getcwd(), '../..', 'data')
    parent_path = os.path.abspath(parent_path)
    file_name = "armc_repeat _position_m.txt"
    path = os.path.join(parent_path, file_name)
    FileOpen.write(qq_m, path)
    return qq

#=================笛卡尔空间规划=================#
def line_force_plan_armt_c():
    #建立规划类
    linePlan = PathPlan.LinePlan()

    # 获取机器人参数
    DH_0 = rp.DHf_armt
    qq_max = rp.q_max_armt
    qq_min = rp.q_min_armt
    linePlan.get_robot_parameter(DH_0, qq_min, qq_max)

    #规划起点和终点
    Xb = np.array([0.43, 0, 0.015, 0, 3.14, 0])
    Xe = Xb + np.array([0.16, 0, 0, 0, 0, 0])
    linePlan.get_begin_end_point(Xb, Xe)

    #获取起点关节角猜测值
    qq_guess = np.array([0, 46, 0, 90, 0, 42, 0])*np.pi/180.0
    linePlan.get_init_guess_joint(qq_guess)

    #时间和周期
    T = 0.01
    linePlan.get_period(T)
    linePlan.get_plan_time(40)

    #求取关节角度
    qq = linePlan.out_joint()
    [Xx, Xv, Xa] = linePlan.out_zyx()

    #合成位置曲线
    num_init = 1001
    num_pos = len(qq)
    num = num_init + num_pos
    t = np.linspace(0, T * (num - 1), num)

    qd = np.zeros([num, 7])
    xd = np.zeros([num, 6])
    xv = np.zeros([num, 6])
    for i in range(num_init):
        qd[i, :] = qq[0, :]
        xd[i, :] = Xx[0, :]
    qd[num_init:] = qq
    xd[num_init:] = Xx
    xv[num_init:] = Xv

    #生成受力曲线
    fd = -5
    Fd = np.zeros([num, 6])
    [Fd[:num_init, 2], _, _] = bf.interp5rdPoly1(0.0, 0.0, 0.0, fd, 0.0, 0.0, 10, 0.01)
    for i in range(num_pos):
        Fd[num_init + i, 2] = -5

    #返回
    xxd = np.zeros([2*num, 6])
    ffd = np.zeros([2*num, 6])
    xxd[:num, :] = xd
    xxd[num:, :] = xd[::-1]
    ffd[:num, :] = Fd
    ffd[num:, :] = Fd[::-1]

    # 绘制关节角
    MyPlot.plot2_nd(t, xd, title="XX", lable='XX')
    MyPlot.plot2_nd(t, xv, title="XV", lable='XV')
    MyPlot.plot2_nd(t, Fd, title="Fd", lable='fd')
    MyPlot.plot2_nd(t, qd, title="qd", lable='qd')

    # 写入文件
    parent_path = os.path.join(os.getcwd(), '../..', 'data/impedance')
    parent_path = os.path.abspath(parent_path)
    pos_path = parent_path + "/armt_line_pos3.txt"
    posision_path = parent_path + "/armt_line_posision3.txt"
    force_path = parent_path + "/armt_line_force3.txt"

    FileOpen.write(xxd, pos_path)
    FileOpen.write(qd, posision_path)
    FileOpen.write(ffd, force_path)
    return qq

def main():
    #直线规划
    #line_plan()
    #force_plan()
    line_force_plan_armc()
    #force_plan_armc_stiff()
    #repeat_positionin_plan()
    #line_force_plan_armt_c()
    print "finish!"

if __name__ == '__main__':
    main()
