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
    pub = rospy.Publisher("/armc/joint_positions_controller/command", Float64MultiArray, queue_size=1)
    rospy.init_node("joint_position_command", anonymous=True)
    rate = rospy.Rate(400) # 10hz

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

        send_data = Float64MultiArray()
        send_data.data = command_data[k,:]
        print send_data.data
        pub.publish(send_data)
        rate.sleep()
        k = k + 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
