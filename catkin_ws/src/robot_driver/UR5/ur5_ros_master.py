#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于TCP\IP通讯连接UR5, 加入ros节点发送消息
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020.10.16
#系统函数

import numpy as np
import threading
import time

import struct
import socket

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class UR5ROSMaster(object):
    def __init__(self, ip):
        self.n = 6
        # 获得连接ip
        self.ip = ip

    def init(self):
        HOST = self.ip
        PORT = 30003
        robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot.settimeout(2.0)
        robot.connect((HOST, PORT))
        self.robot = robot

    #读取关节角度
    def read(self):
        recvData = bytearray(self.robot.recv(1108))

        qq = np.zeros(self.n)
        qq[0] = struct.unpack('!d', recvData[252:260])[0]
        qq[1] = struct.unpack('!d', recvData[260:268])[0]
        qq[2] = struct.unpack('!d', recvData[268:276])[0]
        qq[3] = struct.unpack('!d', recvData[276:284])[0]
        qq[4] = struct.unpack('!d', recvData[284:292])[0]
        qq[5] = struct.unpack('!d', recvData[292:300])[0]
        self.joint_state = qq

    #写入关节角函数:待改进
    def write(self):
        qq = list(self.command)
        data = 'servoj(' + str(qq) + ',0,0,0.06,0.03,300)\n'
        self.robot.send(data)

    #上位机关节角命令灰调函数
    def command_callback(self, msg):
        self.command = msg.data
        self.write()

    def ros_pub(self, pub):
        #发送关节角度
        self.pub = pub
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.read()
            data = JointState()
            data.position = self.joint_state
            self.pub.send(data)
            rate.sleep()

def main():
    # 运行话题
    rospy.init_node('UR5_master_node')

    #建立机器人类
    robot1 = UR5ROSMaster('192.168.0.2')

    #订阅上位机命令
    rospy.Subscriber('/armr/command', Float64MultiArray, robot1.command_callback)

    #建立线程运行回调函数
    t1 = threading.Thread(target=rospy.spin)
    t1.start()

    #发布机器人关节角度状态
    pub = rospy.Publisher("/armr/joint_states", JointState, queue_size=100)
    robot1.ros_pub(pub)

if __name__ == '__main__':
    main()