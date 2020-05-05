#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档接收话题案例
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.1.11

import numpy as  np
from std_msgs.msg import String
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener2():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def listener1():
    rospy.init_node('listener_test')
    rospy.sleep(0.01)
    print "a"
    #data = rospy.wait_for_message("/scan", LaserScan, )
    while not rospy.is_shutdown():
        msg1 = rospy.wait_for_message('/robot2/ft_sensor_topic', WrenchStamped, timeout=None)
        #msg2 = rospy.wait_for_message('/robot1/joint_states', JointState, timeout=None)
        print "msg1: %s" % msg1
        #print "msg2: %s" % msg2
        # qq = np.zeros(7)
        # for i in range(7):
        #     qq[i] = msg2.position[i]
        # print "当前关节角: %s" % qq

    #print(data.ranges)


if __name__ == '__main__':
    listener1()
