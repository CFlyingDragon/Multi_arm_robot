#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于发送关节角度
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.11.6

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

import os
import numpy as np

from robot_python import FileOpen

def talker():
    #建立节点
    pub1 = rospy.Publisher("armc/joint1_effort_controller/command", Float64, queue_size=1)
    pub2 = rospy.Publisher("armc/joint2_effort_controller/command", Float64, queue_size=1)
    pub3 = rospy.Publisher("armc/joint3_effort_controller/command", Float64, queue_size=1)
    pub4 = rospy.Publisher("armc/joint4_effort_controller/command", Float64, queue_size=1)
    pub5 = rospy.Publisher("armc/joint5_effort_controller/command", Float64, queue_size=1)
    pub6 = rospy.Publisher("armc/joint6_effort_controller/command", Float64, queue_size=1)
    pub7 = rospy.Publisher("armc/joint7_effort_controller/command", Float64, queue_size=1)

    rospy.init_node("joint_position_command", anonymous=True)
    rate = rospy.Rate(100) # 10hz

    #读取命令文件
    file_path = os.path.abspath("..")
    file_name = 'data/position.txt'
    path = os.path.join(file_path,file_name)
    command_pos = np.array(FileOpen.read(path))
    #print command_pos.shape()

    #重写数据
    kk = len(command_pos[:, 0])
    n = len(command_pos[0, :])
    print "数据个数：%d" % kk
    print "数据长度：%d" % n
    command_data = np.zeros([kk,n])
    for i in range(kk):
        for j in range(n):
            command_data[i,j] = command_pos[i,j]
            
    k = 0
    while not rospy.is_shutdown():
        if k == kk:
            break
        tip_str = "第 %s 次命令：" % k
        rospy.loginfo(tip_str)

        joint1_data = Float64()
        joint2_data = Float64()
        joint3_data = Float64()
        joint4_data = Float64()
        joint5_data = Float64()
        joint6_data = Float64()
        joint7_data = Float64()

        joint1_data.data = command_data[k, 0]
        joint2_data.data = command_data[k, 1]
        joint3_data.data = command_data[k, 2]
        joint4_data.data = command_data[k, 3]
        joint5_data.data = command_data[k, 4]
        joint6_data.data = command_data[k, 5]
        joint7_data.data = command_data[k, 6]
        print "send data:%s" % command_data[k, :]
        pub1.publish(joint1_data)
        pub2.publish(joint2_data)
        pub3.publish(joint3_data)
        pub4.publish(joint4_data)
        pub5.publish(joint5_data)
        pub6.publish(joint6_data)
        pub7.publish(joint7_data)

        rate.sleep()
        k = k + 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
