#!/usr/bin/env python
# -*-coding:utf-8-*-
# 本文档用于发送关节角度
# 程序员：CYT
# 版权：哈尔滨工业大学(深圳）
# 日期：初稿：2019.11.6

import rospy
from std_msgs.msg import Float64MultiArray

import os
import numpy as np

def talker():
    # 建立节点
    rospy.init_node("joint_position_command", anonymous=True)
    #建立话题
    pub = rospy.Publisher("joint_command", Float64MultiArray, queue_size=1)
    #设置发送频率
    rate = rospy.Rate(100)  # 100hz

    #假设数据
    command_pos = np.zeros([1000,7])

    # 重写数据
    kk = len(command_pos[:, 0])
    n = len(command_pos[0, :])
    command_data = np.zeros([kk, n])
    for i in range(kk):
        for j in range(n):
            command_data[i, j] = command_pos[i, j]

    #主循环中发送数据
    k = 0
    while not rospy.is_shutdown():
        if k == kk:
            break
        tip_str = "第 %s 次命令：" % k
        rospy.loginfo(tip_str)

        send_data = Float64MultiArray()
        send_data.data = command_data[k, :]
        print send_data.data
        #发送数据
        pub.publish(send_data)
        #休眠
        rate.sleep()
        k = k + 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
