#!/usr/bin/env python
# -*-coding:utf-8-*-
# 本文档用于接收信息
# 程序员：陈永厅
# 版权：哈尔滨工业大学
# 日期：初稿：2019.12.12

import os
import numpy as np
from robot_python import FileOpen
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard position %s', data.position)
    rospy.loginfo(rospy.get_caller_id() + 'I heard velocity %s', data.velocity)
    joint_position = np.zeros([1,7])
    joint_position[0,:] = data.position
    joint_velocity = np.zeros([1, 7])
    joint_velocity[0,:] = data.velocity

    path1 = "/home/d/catkin_ws/src/robot_bag/sigle_joint_test/joint_state_position203.txt"
    path2 = "/home/d/catkin_ws/src/robot_bag/sigle_joint_test/joint_state_velocity203.txt"

    #time1 = time.clock()
    FileOpen.write_a(joint_position,path1)
    FileOpen.write_a(joint_velocity, path2)
    #time2 = time.clock()
    # dt = time2 - time1
    # print "rw neet time: %s" % dt

def listener2():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/joint_states', JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def listener1():
    rospy.init_node('get_laser')
    rospy.sleep(2)
    print "a"
    #data = rospy.wait_for_message("/scan", LaserScan, )
    while not rospy.is_shutdown():
        #msg1 = rospy.wait_for_message('six_axis_force_1', Float64MultiArray, timeout=None)
        msg2 = rospy.wait_for_message('/joint_states', JointState, timeout=None)
        #print "msg1: %s" % msg1
        #print "msg2: %s" % msg2
        q1 = msg2.position[0]
        print "joint state position: %s" % q1

    #print(data.ranges)


if __name__ == '__main__':
    listener2()
