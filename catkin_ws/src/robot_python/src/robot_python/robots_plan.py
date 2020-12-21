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
import Kinematics as kin
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei'] # 指定默认字体
plt.rcParams['axes.unicode_minus'] = False # 解决保存图像是负号'-'显示为方块的问题

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

#=================两个UR5机械臂搬运木箱规划:加入手抓=================#
def ur5s_move_object_plan_hand():
    Xb = rp.board_X
    Xe = Xb + np.array([0.0, 0.35, 0.250, 0, 0, 0])
    T = 0.01
    t = 30
    #建立多臂规划类
    robots1 = Robots.RobotsHandMoveObject()
    #获取DH参数
    robots1.get_robot1_paramter(rp.DH0_ur5, rp.q_min_ur5, rp.q_max_ur5)
    robots1.get_robot2_paramter(rp.DH0_ur5, rp.q_min_ur5, rp.q_max_ur5)
    #获取基座到世界坐标系参数
    robots1.get_robots_base_to_world(rp.robot1_base_T, rp.robot2_base_T)

    #加入手抓
    robots1.get_grasp_end(rp.T_grasp)

    #获取抓取点相对于工件坐标系
    T_o_t1 = np.eye(4)
    T_o_t2 = np.eye(4)
    T_o_t1[0:3, 0:3] = np.array([[0, 0, 1],
                                 [-1, 0, 0],
                                 [0, -1, 0.0]])
    T_o_t1[0:3, 3] = np.array([-0.1, 0.0, 0.044])
    T_o_t2[0:3, 0:3] = np.array([[0, 0, -1],
                                 [1, 0, 0],
                                 [0, -1, 0.0]])
    T_o_t2[0:3, 3] = np.array([0.1, 0.0, 0.044])
    robots1.get_robots_tool_to_object(T_o_t1, T_o_t2)

    #输入准备长度
    l = 0.10
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
    file_name1 = "data/robots/robot1_hand_move_position.txt"
    path1 = os.path.join(parent_path, file_name1)
    FileOpen.write(qq1, path1)

    # 写入文件
    file_name2 = "data/robots/robot2_hand_move_position.txt"
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

#=================两个自制机械臂搬运木箱规划=================#
def armct_move_object_plan():
    Xb = rp.ball_X
    Xe = Xb + np.array([0.0, 0.0, 0.100, 0, 0, 0])
    T = 0.01
    t = 30
    #建立多臂规划类
    robots1 = Robots.ArmctMoveObject()
    #获取DH参数
    robots1.get_robot1_paramter(rp.DHfx_armt, rp.q_min_armt, rp.q_max_armt)
    robots1.get_robot2_paramter(rp.DHfx_armc, rp.q_min_armc, rp.q_max_armc)
    #获取基座到世界坐标系参数
    robots1.get_robots_base_to_world(rp.armt_base_T, rp.armc_base_T)

    #获取抓取点相对于工件坐标系
    T_o_t1 = np.eye(4)
    T_o_t2 = np.eye(4)
    T_o_t1[0:3, 0:3] = np.array([[0, 0, -1],
                                 [0, -1, 0],
                                 [-1, 0, 0.0]])
    T_o_t1[0:3, 3] = np.array([0.127, 0.0, 0.000])
    T_o_t2[0:3, 0:3] = np.array([[0, 0, 1],
                                 [0, 1, 0],
                                 [-1, 0, 0.0]])
    T_o_t2[0:3, 3] = np.array([-0.127, 0.0, 0.000])
    robots1.get_robots_tool_to_object(T_o_t1, T_o_t2)

    #输入准备长度
    l = 0.030
    ready_num = 400
    robots1.get_ready_distance(l, ready_num)

    num_s = 500
    robots1.get_move_stop_num(num_s)

    #输入工件规划点
    Xe_list = workpiece_moving_track_line(Xb, Xe, T, t)
    robots1.get_object_plan_list_zyx(Xe_list)

    #获取关节角
    qq1_guess = np.array([0, -45, 0, 85, 0, 50, 0])*np.pi / 180.0
    qq2_guess = np.array([0, -45, 0, 85, 0, 50, 0]) * np.pi / 180.0
    [qq1, qq2] = robots1.put_robots_joint_position(qq1_guess, qq2_guess)

    fr1 = -0.0
    fr2 = 0.0
    fd = -30
    [f1, f2] = robots1.put_expect_force(fr1, fd, fr2)

    # 绘制关节角
    num12 = len(qq1)
    t12 = np.linspace(0, T * (num12 - 1), num12)
    MyPlot.plot2_nd(t12, qq1, title="qq1")
    MyPlot.plot2_nd(t12, qq2, title="qq2")
    MyPlot.plot2_nd(t12, f1, title="f")
    MyPlot.plot2_nd(t12, f2, title="f2")

    # 写入文件
    parent_path = os.path.join(os.getcwd(), '../..')
    parent_path = os.path.abspath(parent_path)
    file_name1 = "data/robots/armct/armt_position.txt"
    path1 = os.path.join(parent_path, file_name1)
    FileOpen.write(qq1, path1)

    # 写入文件
    file_name2 = "data/robots/armct/armc_position.txt"
    path2 = os.path.join(parent_path, file_name2)
    FileOpen.write(qq2, path2)

    file_name3 = "data/robots/armct/armt_force.txt"
    path3 = os.path.join(parent_path, file_name3)
    FileOpen.write(f1, path3)

    # 写入文件
    file_name4 = "data/robots/armct/armc_force.txt"
    path4 = os.path.join(parent_path, file_name4)
    FileOpen.write(f2, path4)

    return [qq1, qq2]

#=================三个自制机械臂搬运木箱规划=================#
def armctr_move_object_plan():
    ## 建立多臂规划类
    robots1 = Robots.robotsMoveObject()
    robots2 = Robots.robotsMoveObject()
    robots3 = Robots.robotsMoveObject()

    #机械臂关节角度
    n1 = 6
    n2 = 7
    n3 = 7

    # 获取DH参数
    robots1.get_robot_paramter(rp.DH0_ur5, rp.q_min_ur5, rp.q_max_ur5)
    robots2.get_robot_paramter(rp.DHfx_armt, rp.q_min_armt, rp.q_max_armt)
    robots3.get_robot_paramter(rp.DHfx_armc, rp.q_min_armc, rp.q_max_armc)

    kin2 = kin.GeneralKinematic(rp.DHfx_armt, rp.q_min_armt, rp.q_max_armt)
    kin3 = kin.GeneralKinematic(rp.DHfx_armc, rp.q_min_armc, rp.q_max_armc)

    # 获取基座到世界坐标系参数
    # Tb1 = rp.robotsr_armr_base_T
    # Tb2 = rp.robotsr_armt_base_T
    # Tb3 = rp.robotsr_armc_base_T
    a = np.sqrt(2)/2
    Tb1 = np.array([[-a, -a, 0, 0.6], [a, -a, 0, 0], [0, 0, 1, 0.020], [0, 0, 0, 1.0]])
    Tb2 = np.array([[1, 0, 0, -0.825], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    Tb3 = np.array([[0, 1, 0, 0], [-1, 0, 0, 1.05], [0, 0, 1, 0], [0, 0, 0, 1.0]])

    robots1.get_robot_base_to_world(Tb1)
    robots2.get_robot_base_to_world(Tb2)
    robots3.get_robot_base_to_world(Tb3)

    ##物体运动轨迹轨迹
    T = 0.01
    t = 10
    Tw_o = np.eye(4)

    xo_1 = np.array([0, 0, 0.2])
    xo_2 = xo_1 + np.array([0, 0, 0.25])
    xo_3 = xo_2 + np.array([0.125, 0, 0])

    x0 = np.zeros(3)

    #搬运段
    [xo_12_array, _, _] = bf.interp5rdPoly(xo_1, x0, x0, xo_2, x0, x0, t, T)
    num_o12 = len(xo_12_array)
    Tw_o12 = np.zeros([num_o12, 4, 4])
    for i in range(num_o12):
        Tw_o[0:3, 3] = xo_12_array[i, :]
        Tw_o12[i, :, :] = Tw_o

    #协同打磨段
    [xo_23_array, _, _] = bf.interp5rdPoly(xo_2, x0, x0, xo_3, x0, x0, t, T)
    num_o23 = len(xo_23_array)
    Tw_o23 = np.zeros([num_o23, 4, 4])
    for i in range(num_o23):
        Tw_o[0:3, 3] = xo_23_array[i, :]
        Tw_o23[i, :, :] = Tw_o

    ##获取抓取点相对于工件坐标系
    To_t1 = np.array([[0, 0, -1, 0.126], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]])
    To_t2 = np.array([[0, 0, 1, -0.126], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1.0]])
    To_t3 = np.array([[0, 1, 0, 0], [0, 0, -1, 0.14], [-1, 0, 0, 0.1], [0, 0, 0, 1.0]])

    xo_t3_1 = np.array([0, 0.14, 0.1])
    xo_t3_2 = xo_t3_1 + np.array([0, 0, -0.2])

    t = 10
    [xo_t3_array, _, _] = bf.interp5rdPoly(xo_t3_1, x0, x0, xo_t3_2, x0, x0, t, T)
    num_t3 = len(xo_t3_array)
    To_t3_array = np.zeros([num_t3, 4, 4])
    for i in range(num_t3):
        To_t3_array[i, :, :] = To_t3
        To_t3_array[i, 0:3, 3] = xo_t3_array[i, :]

    #输入准备长度
    l = 0.040
    t_r = 5
    [l_array, _, _] = bf.interp5rdPoly1(l, 0.0, 0.0, 0.0, 0.0, 0.0, t_r, T)
    num_r = len(l_array)
    To_t1_l_array = np.zeros([num_r, 4, 4])
    To_t2_l_array = np.zeros([num_r, 4, 4])
    To_t3_l_array = np.zeros([num_r, 4, 4])
    for i in range(num_r):
        To_t1_l_array[i, :, :] = To_t1
        To_t1_l_array[i, 0, 3] = To_t1_l_array[i, 0, 3] + l_array[i]
        To_t2_l_array[i, :, :] = To_t2
        To_t2_l_array[i, 0, 3] = To_t2_l_array[i, 0, 3] - l_array[i]
        To_t3_l_array[i, :, :] = To_t3_array[0, :, :]
        To_t3_l_array[i, 1, 3] = To_t3_l_array[i, 1, 3] + l_array[i]

    #获取关节角
    qq1_guess = np.array([45, -120, -120, 60, 90, 0])*np.pi / 180.0
    qq2_guess = np.array([0, 60, 0, 90, 0, -60, 0]) * np.pi / 180.0
    qq3_guess = np.array([0, 30, 0, 30, 0, 30, 0]) * np.pi / 180.0

    #获取机械臂12的关节角
    #准备段关节角
    qq1_1 = robots1.out_robot_joint(qq1_guess, Tw_o12[0, :, :], To_t1_l_array)
    print "机械臂1准备段求取成功！"
    qq2_1 = robots2.out_robot_joint(qq2_guess, Tw_o12[0, :, :], To_t2_l_array)
    print "机械臂2准备段求取成功！"
    #等待加载力
    num_l = 401
    qq1_2 = np.dot(np.ones([num_l, n1]), np.diag(qq1_1[-1, :]))
    qq2_2 = np.dot(np.ones([num_l, n2]), np.diag(qq2_1[-1, :]))
    #将物体搬运到一定高度
    qq1_3 = robots1.out_robot_joint(qq1_2[-1, :], Tw_o12, To_t1)
    print "机械臂1搬运段求取成功！"
    qq2_3 = robots2.out_robot_joint(qq2_2[-1, :], Tw_o12, To_t2)
    print "机械臂2搬运段求取成功！"
    #等待机械臂3准备并加载力
    num_s = num_l + num_r
    qq1_4 = np.dot(np.ones([num_s, n1]), np.diag(qq1_3[-1, :]))
    qq2_4 = np.dot(np.ones([num_s, n2]), np.diag(qq2_3[-1, :]))
    #协同运动段
    qq1_5 = robots1.out_robot_joint(qq1_4[-1, :], Tw_o23, To_t1)
    qq2_5 = robots2.out_robot_joint(qq2_4[-1, :], Tw_o23, To_t2)
    print "机械臂12协同段求取成功！"

    #获取机械臂3的关节角
    #准备段关节角度
    qq3_1 = robots3.out_robot_joint(qq3_guess, Tw_o23[0, :, :], To_t3_l_array)
    print "机械臂3准备段求取成功！"
    #等待机械臂3加载力
    qq3_2 = np.dot(np.ones([num_l, n3]), np.diag(qq3_1[-1, :]))
    #协同运动动
    qq3_3 = robots3.out_robot_joint(qq3_2[-1, :], Tw_o23, To_t3_array)
    print "机械臂3协同段求取成功！"

    ##按时序拼接对应段
    qq1_ = np.concatenate((qq1_1, qq1_2, qq1_3, qq1_4, qq1_5), axis=0)
    qq2_ = np.concatenate((qq2_1, qq2_2, qq2_3, qq2_4, qq2_5), axis=0)
    qq3_ = np.concatenate((qq3_1, qq3_2, qq3_3), axis=0)
    qq1 = np.concatenate((qq1_, qq1_[::-1, :]), axis=0)
    qq2 = np.concatenate((qq2_, qq2_[::-1, :]), axis=0)
    qq3 = np.concatenate((qq3_, qq3_[::-1, :]), axis=0)

    ##规划控制力
    #机械臂2
    num2 = len(qq2)
    Fd2 = np.array([0, 0, -40, 0, 0, 0.0])
    F0 = np.zeros(6)
    f2_r = -np.ones([num_r, 6])*0
    t_l = (num_l-1)*T
    [f2_l, _, _] = bf.interp5rdPoly(F0, F0, F0, Fd2, F0, F0, t_l, T)
    f2_3 = np.dot(np.ones([num2 - 2*num_r - 2*num_l, 6]), np.diag(Fd2))
    ff2 = np.concatenate((f2_r, f2_l, f2_3, f2_l[::-1, :], f2_r[::-1, :]), axis=0)

    #机械臂3
    num3 = len(qq3)
    Fd3 = np.array([0, 0, -5, 0, 0, 0.0])
    f3_r = -0*np.ones([num_r, 6])
    [f3_l, _, _] = bf.interp5rdPoly(F0, F0, F0, Fd3, F0, F0, t_l, T)
    f3_3 = np.dot(np.ones([num3 - 2 * num_r - 2 * num_l, 6]), np.diag(Fd3))
    ff3 = np.concatenate((f3_r, f3_l, f3_3, f3_l[::-1, :], f3_r[::-1, :]), axis=0)

    #存储期望力
    f2 = ff2[:, 2]
    f3 = np.zeros(num2)
    f3[((num2-num3)/2):((num2+num3)/2)] = ff3[:, 2]
    f = np.zeros([num2, 2])
    f[:, 0] = f2
    f[:, 1] = f3

    #存储末端轨迹
    XX_t = np.zeros([num2, 3])
    XX_c = np.zeros([num3, 3])
    for i in range(num2):
        Te = kin2.fkine(qq2[i, :])
        XX_t[i, :] = Te[0:3, 3]

    for i in range(num3):
        Te = kin3.fkine(qq3[i, :])
        XX_c[i, :] = Te[0:3, 3]

    #绘制论文图
    t12 = np.linspace(0, T * (num2 - 1), num2)
    t3 = np.linspace(0, T * (num3 - 1), num3)
    # 绘制数据图
    dpi = 500
    #搬运臂期望位置
    plt.figure(1)
    plt.plot(t12, 1000*XX_t[:, 0], linewidth='2', label='x', color='r', linestyle='--')
    plt.plot(t12, 1000*XX_t[:, 1], linewidth='2', label='y', color='g', linestyle='-.')
    plt.plot(t12, 1000*XX_t[:, 2], linewidth='2', label='z', color='b')
    plt.title(u"搬运机械臂Armt期望轨迹")
    plt.xlabel("t(s)")
    plt.ylabel("X(mm)")
    plt.ylim(-100, 1000)
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    #曲面直线
    plt.figure(2)
    plt.plot(t3, 1000*XX_c[:, 0], linewidth='2', label='x', color='r', linestyle='--')
    plt.plot(t3, 1000*XX_c[:, 1], linewidth='2', label='y', color='g', linestyle='-.')
    plt.plot(t3, 1000*XX_c[:, 2], linewidth='2', label='z', color='b')
    plt.title(u"未知曲面直线运动机械臂期望轨迹")
    plt.xlabel("t(s)")
    plt.ylabel("X(mm)")
    plt.ylim(-100, 1250)
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    #夹持力
    plt.figure(3)
    plt.plot(t12, f2, linewidth='2', label='Fz', color='b')
    plt.title(u"搬运机械臂Armt期望力")
    plt.ylim(-45, 8)
    plt.xlabel("t(s)")
    plt.ylabel("Fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    #曲面力
    plt.figure(4)
    plt.plot(t3, ff3[:, 2], linewidth='2', label='Fz', color='b')
    plt.ylim(-6, 1)
    plt.title(u"未知曲面直线运动机械臂期望力")
    plt.xlabel("t(s)")
    plt.ylabel("Fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素
    plt.show()

    #计算机械臂1被动受力
    ff1 = -ff2

    #绘制关节角

    MyPlot.plot2_nd(t12, qq1, title="qq1", lable="q_")
    MyPlot.plot2_nd(t12, qq2, title="qq2", lable="q_")
    MyPlot.plot2_nd(t3, qq3, title="qq3", lable="q_")

    #绘制末端位置
    MyPlot.plot2_nd(t12, XX_t, title="XX_t", lable="X")
    MyPlot.plot2_nd(t3, XX_c, title="XX_c", lable="X")

    print "对比：", len(ff1), num2, len(ff3), num3
    MyPlot.plot2_nd(t12, ff1, title="F1", lable="F")
    MyPlot.plot2_nd(t12, ff2, title="F2", lable="F")
    MyPlot.plot2_nd(t3, ff3, title="F3", lable="F")

    MyPlot.plot2_nd(t12, f, title="F", lable="F")

    #写入位置
    parent_path = os.path.join(os.getcwd(), '../..')
    parent_path = os.path.abspath(parent_path)
    file_force = "data/robots/armctr/expect_force.txt"
    path_force = os.path.join(parent_path, file_force)
    FileOpen.write(f, path_force)

    file_pos1 = "data/robots/armctr/armr_position.txt"
    path_pos1 = os.path.join(parent_path, file_pos1)
    FileOpen.write(qq1, path_pos1)

    file_pos2 = "data/robots/armctr/armt_position.txt"
    path_pos2 = os.path.join(parent_path, file_pos2)
    FileOpen.write(qq2, path_pos2)

    file_pos3 = "data/robots/armctr/armc_position.txt"
    path_pos3 = os.path.join(parent_path, file_pos3)
    FileOpen.write(qq3, path_pos3)

    #写入写入力
    file_f1 = "data/robots/armctr/armr_force.txt"
    path_f1 = os.path.join(parent_path, file_f1)
    FileOpen.write(ff1, path_f1)

    file_f2 = "data/robots/armctr/armt_force.txt"
    path_f2 = os.path.join(parent_path, file_f2)
    FileOpen.write(ff2, path_f2)

    file_f3 = "data/robots/armctr/armc_force.txt"
    path_f3 = os.path.join(parent_path, file_f3)
    FileOpen.write(ff3, path_f3)

    return 1

def main():
    # 求取规划轨迹
    #robots_polish_object_plan()

    #搬运物体规划
    #ur5s_move_object_plan()

    # 搬运物体规划:加入手抓
    #armct_move_object_plan()

    #三臂规划
    armctr_move_object_plan()

    print "finish!"


if __name__ == '__main__':
    # main()
    armctr_move_object_plan()
