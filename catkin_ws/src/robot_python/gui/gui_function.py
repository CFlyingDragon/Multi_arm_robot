#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于界面相关函数
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.4.26
#系统函数
import os
import numpy as np
from math import pi
import numpy.linalg as nla

#ros相关函数模块
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

#自定义函数模块
from robot_python import FileOpen
from robot_python import JointPlan as jp
from robot_python import RobotParameter as rp
from robot_python import PathPlan as pap
from robot_python import EndPoint as enp
from robot_python import Kinematics as kin


#===============================话题发送和订阅================================#
#发送两个ur5机械臂位置
def publisher2ur5(path1,path2):
    #建立节点
    pub1 = rospy.Publisher("/robot1/ur5_position_controller/command",
                           Float64MultiArray, queue_size=1)
    pub2 = rospy.Publisher("/robot2/ur5_position_controller/command",
                           Float64MultiArray, queue_size=1)
    rospy.init_node("joint_position_command", anonymous=True)
    rate = rospy.Rate(200) # 10hz

    #读取命令文件
    command_pos1 = np.array(FileOpen.read(path1))
    command_pos2 = np.array(FileOpen.read(path2))

    #重写数据
    kk1 = len(command_pos1[:, 0])
    kk2 = len(command_pos2[:, 0])
    n1 = len(command_pos1[0, :])
    n2 = len(command_pos2[0, :])
    print "数据个数：%d" % kk1
    print "数据长度：%d" % n2
    command_data1 = np.zeros([kk1,n1])
    command_data2 = np.zeros([kk2, n2])
    for i in range(kk1):
        for j in range(n1):
            command_data1[i,j] = command_pos1[i,j]
            command_data2[i, j] = command_pos2[i, j]

    #发送话题
    k = 0
    while not rospy.is_shutdown():
        if k == kk1:
            break
        tip_str = "第 %s 次命令：" % k
        rospy.loginfo(tip_str)

        send_data1 = Float64MultiArray()
        send_data2 = Float64MultiArray()
        send_data1.data = command_data1[k,:]
        send_data2.data = command_data2[k, :]
        print send_data1.data
        pub1.publish(send_data1)
        pub2.publish(send_data2)
        rate.sleep()
        k = k + 1

#发送单个机械臂关节角度
def publisher(data_path, pub_path):
    # 建立节点
    pub = rospy.Publisher(pub_path, Float64MultiArray, queue_size=1)
    rospy.init_node("joint_position_command", anonymous=True)
    rate = rospy.Rate(100)  # 100hz

    # 读取命令文件
    command_pos = np.array(FileOpen.read(data_path))

    # 重写数据
    kk = len(command_pos[:, 0])
    n = len(command_pos[0, :])
    command_data = np.zeros([kk, n])
    for i in range(kk):
        for j in range(n):
            command_data[i, j] = command_pos[i, j]
    #发送话题
    k = 0
    while not rospy.is_shutdown():
        if k == kk:
            break
        tip_str = "第 %s 次命令：" % k
        rospy.loginfo(tip_str)

        send_data = Float64MultiArray()
        send_data.data = command_data[k, :]
        pub.publish(send_data)
        rate.sleep()
        k = k + 1

#三臂机器人关节角位置发送
def publisher_3arms(data_path1,data_path2,data_path3):
    #建立节点
    pub1 = rospy.Publisher("/robot1/ur5_position_controller/command",
                           Float64MultiArray, queue_size=1)
    pub2 = rospy.Publisher("/robot2/ur5_position_controller/command",
                           Float64MultiArray, queue_size=1)
    pub3 = rospy.Publisher("/robot3/armc_position_controller/command",
                           Float64MultiArray, queue_size=1)
    rospy.init_node("joint_position_command", anonymous=True)
    rate = rospy.Rate(100) # 100hz

    #读取命令文件
    command_pos1 = np.array(FileOpen.read(data_path1))
    command_pos2 = np.array(FileOpen.read(data_path2))
    command_pos3 = np.array(FileOpen.read(data_path3))

    #重写数据
    kk1 = len(command_pos1[:, 0])
    kk2 = len(command_pos2[:, 0])
    kk3 = len(command_pos3[:, 0])
    n1 = len(command_pos1[0, :])
    n2 = len(command_pos2[0, :])
    n3 = len(command_pos3[0, :])
    command_data1 = np.zeros([kk1, n1])
    command_data2 = np.zeros([kk2, n2])
    command_data3 = np.zeros([kk3, n3])
    for i in range(kk1):
        for j in range(n1):
            command_data1[i, j] = command_pos1[i, j]
    for i in range(kk2):
        for j in range(n2):
            command_data2[i, j] = command_pos2[i, j]
    for i in range(kk3):
        for j in range(n3):
            command_data3[i, j] = command_pos3[i, j]
    #发送话题
    k = 0
    while not rospy.is_shutdown():
        if k == kk1:
            break
        tip_str = "第 %s 次命令：" % k
        rospy.loginfo(tip_str)

        send_data1 = Float64MultiArray()
        send_data2 = Float64MultiArray()
        send_data3 = Float64MultiArray()
        send_data1.data = command_data1[k, :]
        send_data2.data = command_data2[k, :]
        send_data3.data = command_data3[k, :]
        pub1.publish(send_data1)
        pub2.publish(send_data2)
        pub3.publish(send_data3)
        rate.sleep()
        k = k + 1

# ===============================规划模块================================#
#主界面中，用于规划关节空间点到点的运动
def q_joint_space_plan(qq_b, qq_e, T=0.01):
    '''
    :param qq_b: 规划起始点
    :param qq_e: 规划终止点
    :return:
    '''
    #给定运动速度5rmp/min
    vel = 5*2*pi/60.0
    #关节角度转换到弧度单位rad
    qqb = qq_b
    qqe = qq_e
    #计算总时长

    q_max = np.max(np.abs(qqe - qqb))
    t = q_max/vel
    if(t < 10):
        t = 10
    [qq,qv,qa] = jp.pos1_to_pos2(qqb, qqe, T, t)
    return [qq, qv, T]

#关节空间规划,给定时间
def q_joint_space_plan_time(qq_b, qq_e, T=0.01,t=10):
    '''
    :param qq_b:
    :param qq_e:
    :param T:
    :param t:
    :return:
    '''
    [qq, qv, qa] = jp.pos1_to_pos2(qq_b, qq_e, T, t)
    return [qq, qv, qa]

def get_begin_point(qr_init,robot):
    '''
    :param qr_init:
    :param flag:
    :return:
    '''
    [DH0, q_max, q_min] = get_robot_parameter(robot)
    xe = kin.fkine_euler(DH0, qr_init)
    return xe

#规划子界面中，用于规划给定起点和圆心的规划
def cir_plan(qr_init,Xb,rc,T,t,flag):
    '''
    :param qr_init:
    :param Xb:
    :param rc:
    :param T:
    :param t:
    :param flag:
    :return:
    '''
    #默认参数armc
    DH0 = rp.DHf_armc
    qq_max = rp.q_max_armc
    qq_min = rp.q_min_armc
    #勾选时，ur5
    if flag == True:
        DH0 = rp.DH0_ur5
        qq_max = rp.q_max_ur5
        qq_min = rp.q_min_ur5
    #建立末端点
    X0_e = enp.circleEnd_plan(Xb,rc,T*10,t)
    #调用规划函数
    [qq, qv_, qa_] = pap.multipoint_plan_position_w(qr_init, X0_e, T, DH0, qq_max, qq_min)
    return [X0_e,qq]

#规划子界面中，用于规划给定起点和圆心的规划
def line_plan(qr_init,Xb,Xe,T,t,flag):
    '''
    :param qr_init:
    :param Xb:
    :param rc:
    :param T:
    :param t:
    :param flag:
    :return:
    '''
    qq_init = np.copy(qr_init)
    print "qq_init",qq_init
    #默认参数armc
    DH0 = rp.DHf_armc
    qq_max = rp.q_max_armc
    qq_min = rp.q_min_armc
    #勾选时，ur5
    if flag == 1:
        DH0 = rp.DH0_ur5
        qq_max = rp.q_max_ur5
        qq_min = rp.q_min_ur5
        qq_init = np.copy(qr_init[0:6])
    #建立末端点
    X0_e = enp.lineEnd_plan(Xb,Xe,T*10,t)
    #调用规划函数
    [qq, qv_, qa_] = pap.multipoint_plan_position_w(qq_init, X0_e, T, DH0, qq_max, qq_min)
    return [X0_e,qq]

#===============================自适应阻抗控制================================#
def get_robot_parameter(flag_str):
    # 机械臂是armc
    if(flag_str=='armc'):
        DH0 = rp.DHfa_armc
        qq_max = rp.q_max_armc
        qq_min = rp.q_min_armc
    #机械臂是UR5
    elif(flag_str=='ur5'):
        DH0 = rp.DH0_ur5
        qq_max = rp.q_max_ur5
        qq_min = rp.q_min_ur5
    elif(flag_str=='armt'):
        DH0 = rp.DHf_armt
        qq_max = rp.q_max_armc
        qq_min = rp.q_min_armc
    elif(flag_str=='armc_actor'):
        DH0 = rp.DHf_armt
        qq_max = rp.q_max_armc
        qq_min = rp.q_min_armc
    else:
        print '输入参数错误'
        return -1
    return [DH0, qq_max,   qq_min]

#============================关节角数据采集处理==============================#
#解决仿真中采集个关节角间接性出现零的问题
def joint_pos_Pretreatment(qq,q_list,T):
    '''
    :param qq:当前时刻采集到的关节值
    :param qq_list:前三个时刻的关节值
    :return:处理后的关节值
    '''
    #用抛物线预测下一时刻的关节角
    n = len(qq)
    qq_list = np.copy(q_list[:,0:n])
    #计算评估系数
    b = (3*qq_list[2,:] - 2*qq_list[1,:] - qq_list[0,:])/T*0.5

    qq_p = np.zeros(n)
    for i in range(n):
        if (abs(b[i]) > 5):
            b[i] = b[i] / abs(b[i])
        if (abs((qq[i] - qq_list[-1,i])/(1 + abs(qq_list[-1,i]))) > pow(10,-4)
            and abs(qq[i]) < pow(10,-5)):
            qq_p[i] = qq_list[-1,i] + b[i]*T
        else:
            qq_p[i] = qq[i]
    return qq_p
#给定地址，读取数据
def read_data(path):
    '''
    :param path:
    :return:
    '''
    # 读取命令文件
    command_pos = np.array(FileOpen.read(path))

    # 重写数据
    kk = len(command_pos[:, 0])
    n = len(command_pos[0, :])
    command_data = np.zeros([kk, n])
    for i in range(kk):
        for j in range(n):
            command_data[i, j] = command_pos[i, j]
    return command_data