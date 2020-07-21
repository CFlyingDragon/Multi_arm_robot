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

#=================工件轨迹规划-直线规划=================#
def workpiece_moving_track_line(Xb, Xe, T, t):
    '''
    :return:
    '''
    z = np.zeros(6)

    #采用5次多项式平滑轨迹
    [Xw, Xvw, Xaw] = bf.interp5rdPoly(Xb, z, z, Xe, z, z, t, T)
    return Xw

#=================两个UR5机械臂搬运木箱规划=================#
def ur5s_move_object_plan():
    Xb = rp.Wooden_X
    Xe = Xb + np.array([0.0, 0.15, 0.150, 0, 0, 0])
    T = 0.01
    t = 30
    #建立多臂规划类
    robots1 = Robots.RobotsMoveObject()
    #获取DH参数
    robots1.get_robot1_paramter(rp.DH0_hand_ur5, rp.q_min_ur5, rp.q_max_ur5)
    robots1.get_robot2_paramter(rp.DH0_hand_ur5, rp.q_min_ur5, rp.q_max_ur5)
    #获取基座到世界坐标系参数
    robots1.get_robots_base_to_world(rp.robot1_base_T, rp.robot2_base_T)

    #获取抓取点相对于工件坐标系
    T_o_t1 = np.eye(4)
    T_o_t2 = np.eye(4)
    T_o_t1[0:3, 0:3] = np.array([[0, 0, 1],
                                 [-1, 0, 0],
                                 [0, -1, 0.0]])
    T_o_t1[0:3, 3] = np.array([-0.0485, 0.0, 0.050])
    T_o_t2[0:3, 0:3] = np.array([[0, 0, -1],
                                 [1, 0, 0],
                                 [0, -1, 0.0]])
    T_o_t2[0:3, 3] = np.array([0.0485, 0.0, 0.050])
    robots1.get_robots_tool_to_object(T_o_t1, T_o_t2)

    #输入准备长度
    l = 0.080
    ready_num = 400
    robots1.get_ready_distance(l, ready_num)


    #输入工件规划点
    Xe_list = workpiece_moving_track_line(Xb, Xe, T, t)
    robots1.get_object_plan_list_zyx(Xe_list)

    #获取关节角
    qq1_guess = np.array([-90, -120, -120, 60, 90, 0])*np.pi / 180.0
    qq2_guess = np.array([-90, -60, 120, 120, -90, 0]) * np.pi / 180.0
    [qq1, qq2] = robots1.put_robots_joint_position(qq1_guess, qq2_guess)

    # 绘制关节角
    num12 = len(qq1)
    t12 = np.linspace(0, T * (num12 - 1), num12)
    MyPlot.plot2_nd(t12, qq1, title="qq1")
    MyPlot.plot2_nd(t12, qq2, title="qq2")

    # 写入文件
    parent_path = os.path.join(os.getcwd(), '../..')
    parent_path = os.path.abspath(parent_path)
    file_name1 = "data/robots/robot1_move_position2.txt"
    path1 = os.path.join(parent_path, file_name1)
    FileOpen.write(qq1, path1)

    # 写入文件
    file_name2 = "data/robots/robot2_move_position2.txt"
    path2 = os.path.join(parent_path, file_name2)
    FileOpen.write(qq2, path2)

    return [qq1, qq2]

def go_to_world_pos():
    Xb = rp.Wooden_X
    phi =np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0.0]])
    Xb[3:6] = bf.rot2euler_zyx(phi)
    # 建立多臂规划类
    robots1 = Robots.robot_move_to_world_pos()
    # 获取DH参数
    robots1.get_robot_paramter(rp.DH0_ur5, rp.q_min_ur5, rp.q_max_ur5)
    # 获取基座到世界坐标系参数
    robots1.get_robots_base_to_world_zyx(rp.robot1_base)
    qq_guess = np.zeros(6)
    qr = robots1.put_robot1_joint_position_zyx(Xb, qq_guess)
    qq = qr + rp.DH0_ur5[:, 0]
    return qq

#=================三个机械臂打磨规划=================#
def robots_polish_object_plan():
    Xb = rp.Wooden_X
    Xe = Xb + np.array([0.00, 0.2, 0.100, 0, 0, 0])
    T = 0.01
    t = 30
    #建立多臂规划类
    robots1 = Robots.RobotsPolishObject()
    #获取DH参数
    robots1.get_robot1_paramter(rp.DH0_ur5, rp.q_min_ur5, rp.q_max_ur5)
    robots1.get_robot2_paramter(rp.DH0_ur5, rp.q_min_ur5, rp.q_max_ur5)
    robots1.get_robot3_paramter(rp.DH0_ur5, rp.q_min_ur5, rp.q_max_ur5)
    #获取基座到世界坐标系参数
    robots1.get_robots_base_to_world(rp.robot1_base_T, rp.robot2_base_T, rp.robot3_base_T)

    #获取抓取点相对于工件坐标系
    T_o_t1 = np.eye(4)
    T_o_t2 = np.eye(4)
    T_o_t1[0:3, 0:3] = np.array([[0, 0, 1],
                                 [-1, 0, 0],
                                 [0, -1, 0.0]])
    T_o_t1[0:3, 3] = np.array([-0.048, 0.0, 0.050])
    T_o_t2[0:3, 0:3] = np.array([[0, 0, -1],
                                 [1, 0, 0],
                                 [0, -1, 0.0]])
    T_o_t2[0:3, 3] = np.array([0.048, 0.0, 0.050])
    robots1.get_robot12_tool_to_object(T_o_t1, T_o_t2)

    #输入准备长度
    l = 0.060
    ready_num = 400
    robots1.get_ready_distance(l, ready_num)

    #输入工件规划点
    Xe_list = workpiece_moving_track_line(Xb, Xe, T, t)

    robots1.get_object_plan_list_zyx(Xe_list)

    #输入机械臂3的打磨轨迹(相对于工件坐标系)
    #打磨起始点
    Xo_t3_init = np.array([-0.05, 0.0, 0.099, 0, np.pi, 0])
    #打磨终点
    Xo_t3_end = np.array([0.05, 0.0, 0.099, 0, np.pi, 0])
    #打磨轨迹
    t3 = 20
    Xo_t3 = workpiece_moving_track_line(Xo_t3_init, Xo_t3_end, T, t3)

    #输入工件相对位置
    robots1.get_robot3_tool_to_object_zyx(Xo_t3)

    #获取末端位姿
    [X1, X2, X3] = robots1.put_robots_tool_position_zyx()
    # 笛卡尔单变量随时间绘图
    # 绘制关节角
    T = 0.01
    num12 = len(X1)
    t12 = np.linspace(0, T * (num12 - 1), num12)

    num3 = len(X3)
    t3 = np.linspace(0, T * (num3 - 1), num3)

    #获取关节角
    qq1_guess = np.array([-90, -120, -120, 60, 90, 0])*np.pi / 180.0
    qq2_guess = np.array([-90, -60, 120, 120, -90, 0]) * np.pi / 180.0
    qq3_guess = np.array([0, -90, -90, -90, 90, 0]) * np.pi / 180.0
    [qq1, qq2, qq3] = robots1.put_robots_joint_position(qq1_guess, qq2_guess, qq3_guess)

    #获取规划的力
    Fd = np.zeros(6)
    Fd[2] = -10
    fd3 = robots1.put_polish_force(Fd)

    # 绘制关节角
    num12 = len(qq1)
    t12 = np.linspace(0, T * (num12 - 1), num12)

    num3 = len(qq3)
    t3 = np.linspace(0, T * (num3 - 1), num3)

    # 笛卡尔单变量随时间绘图
    MyPlot.plot2_nd(t12, qq1, title="qq1")
    MyPlot.plot2_nd(t12, qq2, title="qq2")
    MyPlot.plot2_nd(t3, qq3, title="qq3")
    MyPlot.plot2_nd(t3, fd3, title="fd3")

    # 写入文件
    parent_path = os.path.join(os.getcwd(), '../..')
    parent_path = os.path.abspath(parent_path)
    file_name1 = "data/robots/robot1_polish_position1.txt"
    path1 = os.path.join(parent_path, file_name1)
    FileOpen.write(qq1, path1)

    # 写入文件
    file_name2 = "data/robots/robot2_polish_position1.txt"
    path2 = os.path.join(parent_path, file_name2)
    FileOpen.write(qq2, path2)

    # 写入文件
    file_name3_qq = "data/robots/robot3_polish_position1.txt"
    path3_qq = os.path.join(parent_path, file_name3_qq)
    FileOpen.write(qq3, path3_qq)

    # 写入文件
    file_name3_f = "data/robots/robot3_polish_force1.txt"
    path3_f = os.path.join(parent_path, file_name3_f)
    FileOpen.write(fd3, path3_f)

    return [qq1, qq2, qq3, fd3]

def main():
    # 求取规划轨迹
    #robots_polish_object_plan()

    #搬运物体规划
    ur5s_move_object_plan()

    print "finish!"


if __name__ == '__main__':
    main()
