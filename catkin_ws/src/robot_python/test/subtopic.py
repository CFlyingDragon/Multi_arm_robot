#!/usr/bin/env python
# -*-coding:utf-8-*-
# 本文档用于接收信息
# 程序员：CYT
# 版权：哈尔滨工业大学(深圳）
# 日期：初稿：2019.12.12

import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray

def callback(data):
    print "msg:", data

def listener1():
    #建立节点
    rospy.init_node('listener', anonymous=True)
    #订阅话题
    rospy.Subscriber('joint_command', Float64MultiArray, callback)
    #调用回调函数，并阻塞
    rospy.spin()

def listener2():
    #用循环来订阅所有数据
    while not rospy.is_shutdown():
        #订阅话题
        msg = rospy.wait_for_message('joint_command', Float64MultiArray, timeout=None)
        print "msg: %s" % msg

if __name__ == '__main__':
    #运行程序1
    #listener1()
    #运行程序2
    listener2()
