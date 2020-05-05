#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于发送关节角度
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.11.6

import rospy
from std_msgs.msg import Float64MultiArray

import os
import numpy as np

from robot_python import FileOpen

def talker():
    #建立节点
    #pub = rospy.Publisher("/armc/armc_position_controller/command", Float64MultiArray, queue_size=1)
    # pub = rospy.Publisher("/armc/armc_effort_controller/command", Float64MultiArray, queue_size=1)
    # pub1 = rospy.Publisher("/robot1/armc_position_controller/command", Float64MultiArray, queue_size=1)
    # pub2 = rospy.Publisher("/robot2/armc_position_controller/command", Float64MultiArray, queue_size=1)
    # pub3 = rospy.Publisher("/robot3/armc_position_controller/command", Float64MultiArray, queue_size=1)
    pub1 = rospy.Publisher("/robot1/ur5_position_controller/command", Float64MultiArray, queue_size=1)
    pub2 = rospy.Publisher("/robot2/ur5_position_controller/command", Float64MultiArray, queue_size=1)
    rospy.init_node("joint_position_command", anonymous=True)
    rate = rospy.Rate(200) # 10hz

    #读取命令文件
    file_path = os.path.abspath("..")
    file_name1 = 'data/ur5_position1.txt'
    file_name2 = 'data/ur5_position2.txt'
    path1 = os.path.join(file_path,file_name1)
    path2 = os.path.join(file_path, file_name2)
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
        # pub3.publish(send_data)
        rate.sleep()
        k = k + 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
