#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于界面相关函数
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.4.22

import rospy
from std_msgs.msg import Float64MultiArray

import os
import numpy as np

#===============================读写文件模块================================#
# 读取数据
def read(path):
    ''' read 函数用于读取数据,数据由单空格隔开
        输入参数：path 读取文件的路径
        返回参数：数组，列表格式
    '''
    read_data = []
    with open(path, 'r') as file_to_read:
        while True:
            line = file_to_read.readline()  # 整行读取数据
            if not line:
                break

            line = line.split(" ")  # 用空格拆分数据

            line = line[:-1]  # 切片去除末尾转行符
            read_data.append(line)
    read_data = read_data
    return read_data

# 将数据写入文件，用单空格分隔数据，末尾转行
def write(write_data, path):
    ''' write函数用于写入矩阵型数据，共两个参数
        第一个参数为所需写入的数据，第二格参数为写入文件路径和名字
        文件为用单空格分隔数据，末尾转行
        写入模式为添加模式
    '''
    with open(path, 'w') as file_to_write:
        print ('开始读写')
        write_data = np.array(write_data)
        [row, col] = write_data.shape
        for i in range(row):
            for j in range(col):
                data = write_data[i, j]
                file_to_write.write(str(data))
                file_to_write.write(' ')  # 用空格分隔数据
            file_to_write.write('\n')  # 末尾转行符

# 将数据写入文件，用单空格分隔数据，末尾转行
def write_a(write_data, path):
    ''' write函数用于写入矩阵型数据，共两个参数
        第一个参数为所需写入的数据，第二格参数为写入文件路径和名字
        文件为用单空格分隔数据，末尾转行
        写入模式为添加模式
    '''
    with open(path, 'a') as file_to_write:
        print ('开始读写')
        write_data = np.array(write_data)
        [row, col] = write_data.shape
        for i in range(row):
            for j in range(col):
                data = write_data[i, j]
                file_to_write.write(str(data))
                file_to_write.write(' ')  # 用空格分隔数据
            file_to_write.write('\n')  # 末尾转行符

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
    command_pos1 = np.array(read(path1))
    command_pos2 = np.array(read(path2))

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
    command_pos = np.array(read(data_path))

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
    command_pos1 = np.array(read(data_path1))
    command_pos2 = np.array(read(data_path2))
    command_pos3 = np.array(read(data_path3))

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
