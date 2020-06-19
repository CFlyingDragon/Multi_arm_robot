#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于多机械臂规划程序
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020/4/16
import numpy as np
import BaseFunction as bf
import Robots
import RobotParameter as rp

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
    Xe = Xb + np.array([0, 0, 0.100, 0, 0, 0])
    T = 0.01
    t = 30
    #建立多臂规划类
    robots1 = Robots.RobotsMoveObject()
    #获取DH参数
    robots1.get_robot1_paramter(rp.DH0_ur5, rp.q_min_ur5, rp.q_max_ur5)
    robots1.get_robot2_paramter(rp.DH0_ur5, rp.q_min_ur5, rp.q_max_ur5)
    #获取基座到世界坐标系参数
    robots1.get_robots_base_to_world(rp.robot1_base_T, rp.robot2_base_T)

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
    robots1.get_robots_tool_to_object(T_o_t1, T_o_t2)

    #输入准备长度
    l = 0.040
    ready_num = 400
    robots1.get_ready_distance(l, ready_num)


    #输入工件规划点
    Xe_list = workpiece_moving_track_line(Xb, Xe, T, t)
    robots1.get_object_plan_list_zyx(Xe_list)

    #获取关节角
    qq1_guess = np.array([-90, -120, -120, 60, 90, 0])*np.pi / 180.0
    qq2_guess = np.array([-90, -60, 120, 120, -90, 0]) * np.pi / 180.0
    [qq1, qq2] = robots1.put_robots_joint_position(qq1_guess, qq2_guess)

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


def main():
    [qq1, qq2] = ur5s_move_object_plan()

if __name__ == '__main__':
    main()

