#!/usr/bin/env python
# -*-coding:utf-8-*-
# 本文档用于关节状态，并写入文件
# 程序员：陈永厅
# 版权：哈尔滨工业大学(深圳)
# 日期：2020年6月1号

import numpy as np
import time

import rospy
from sensor_msgs.msg import JointState

#文件写入文件地址
dir_path = "/home/d/catkin_ws/src/robot_bag/joint_states"
file_name = "/hanliang4.txt"
path = dir_path + file_name

#给定关节自由度
n = 7

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard position %s', data.position)
    t1 = time.clock()
    with open(path, 'a') as file_to_write:
        for i in range(n):
            file_to_write.write(str(data.position[i]))
            file_to_write.write(' ')  # 用空格分隔数据
        file_to_write.write('\n')  # 末尾转行符
    t2 = time.clock()
    print "写入文件所需要时间：", t2 - t1

def listener():
    print "开始运行！"
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
