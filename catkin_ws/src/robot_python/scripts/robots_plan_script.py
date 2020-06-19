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
from robot_python import robots_plan
from robot_python import RobotParameter as rp
from robot_python import FileOpen
from robot_python import MyPlot
from robot_python import Robots

def move_object_plan():
    #求取规划轨迹
    [qq1, qq2] = robots_plan.ur5s_move_object_plan()

    #绘制关节角
    T = 0.01
    num = len(qq1)
    t = np.linspace(0, T*(num - 1), num)

    #笛卡尔单变量随时间绘图
    for i in range(6):
        i_string = "qq1_" + str(i+1) + "plot"
        MyPlot.plot2d(t, qq1[:, i], i_string)
        i_string = "qq2_" + str(i+1) + "plot"
        MyPlot.plot2d(t, qq2[:, i], i_string)

    #写入文件
    parent_path = os.path.abspath('..')
    file_name1 = "data/robots/robot1_position.txt"
    path1 = os.path.join(parent_path, file_name1)
    FileOpen.write(qq1, path1)

    #写入文件
    file_name2 = "data/robots/robot2_position.txt"
    path2 = os.path.join(parent_path, file_name2)
    FileOpen.write(qq2, path2)

def go_world_pos():
    qr = robots_plan.go_to_world_pos()
    print qr
    print "qr:",np.around(qr*180/3.14)

def main():
    ##搬运物体规划
    move_object_plan()
    ##定点测试
    #go_world_pos()
    print 'finish'


if __name__ == '__main__':
    main()

