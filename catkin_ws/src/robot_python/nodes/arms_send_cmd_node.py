#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于发送关节角度
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.1.14

import rospy
from std_msgs.msg import Float64MultiArray

import os
import numpy as np

from robot_python import FileOpen

def talker():
    #建立节点
    rospy.init_node('arms_command_node')
    pub1 = rospy.Publisher('/robot1/armc_position_controller/command', Float64MultiArray, queue_size=10)
    pub2 = rospy.Publisher('/robot2/armc_position_controller/command', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(100) # 10hz

    #读取命令文件
    file_path = os.path.abspath("..")
    file_name1 = 'data/position1.txt'
    path1 = os.path.join(file_path,file_name1)
    command_pos1 = np.array(FileOpen.read(path1))

    file_name2 = 'data/position2.txt'
    path2 = os.path.join(file_path, file_name2)
    command_pos2 = np.array(FileOpen.read(path2))
    #print command_pos.shape()

    #重写数据
    kk = len(command_pos1[:, 0])
    n = len(command_pos1[0, :])
    print "数据个数：%d" % kk
    print "数据长度：%d" % n
    command_data1 = np.zeros([kk,n])
    command_data2 = np.zeros([kk, n])
    for i in range(kk):
        for j in range(n):
            command_data1[i,j] = command_pos1[i,j]
            command_data2[i,j] = command_pos2[i, j]
            
    k = 0
    while not rospy.is_shutdown():
        if k == kk:
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

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
