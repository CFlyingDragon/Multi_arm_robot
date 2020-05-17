#!/usr/bin/env python
# -*-coding:utf-8-*-
# 本文档用于接收信息
# 程序员：CYT
# 版权：哈尔滨工业大学(深圳）
# 日期：初稿：2019.12.12

import numpy as np
#ros模块
import rospy
from std_msgs.msg import Float64MultiArray
#线程
import threading

class SubTopic(object):
    #定义全局变量
    qq = np.zeros(7)
    F = np.zeros(6)
    def __init__(self,flag = True):
        self.init()
        self.flag = flag
    def init(self):
        #调用第一个函数
        if(self.flag==True):
            print "第一种方式，多线程处理"
            self.listener1()
        else:
            print "第二中方式处理"
            self.listener2()

    def thread_spin(self):
        rospy.spin()

    def callback1(self, data):
        print "msg1:", data

    def callback2(self, data):
        print "msg2:", data

    def listener1(self):
        # 建立节点
        rospy.init_node('listener', anonymous=True)
        # 订阅话题
        rospy.Subscriber('joint_command1', Float64MultiArray, self.callback1)
        rospy.Subscriber('joint_command2', Float64MultiArray, self.callback2)
        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        t1.start()
        #处理其他函数
        while not rospy.is_shutdown():
            #处理其他函数
            print "处理其他函数！"

    def listener2(self):
        # 用循环来订阅所有数据
        while not rospy.is_shutdown():
            # 订阅话题
            msg1 = rospy.wait_for_message('joint_command1', Float64MultiArray, timeout=None)
            print "msg1: %s" % msg1
            msg2 = rospy.wait_for_message('joint_command2', Float64MultiArray, timeout=None)
            print "msg2: %s" % msg2
            #处理其他函数
            print "处理其他函数！"

