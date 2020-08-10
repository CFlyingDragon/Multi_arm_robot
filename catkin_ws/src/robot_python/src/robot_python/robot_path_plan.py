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
    Xb = np.array([0.45, 0, 0.03, 0, 3.14, 0])
    Xe = Xb + np.array([0.1, 0, 0, 0, 0, 0])
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


def main():
    #直线规划
    line_plan()

    print "finish!"


if __name__ == '__main__':
    main()
