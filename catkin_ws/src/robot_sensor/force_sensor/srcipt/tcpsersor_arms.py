#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于传感器通,加入ros节点发送消息
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
from geometry_msgs.msg import WrenchStamped

#建立一个通用的类
class TcpSensor(object):
    def __init__(self, ip, pub):
        self.ip = ip
        self.pub = pub
        self.set_connect_thread()
        self.set_pub_node_thread()

    def set_connect_thread(self):
        # 建立线程接收六维力
        str1 = "t1_" + self.ip[-3:]
        t1 = threading.Thread(target=self.connect, name=str1)
        t1.start()
        time.sleep(5)

    def set_pub_node_thread(self):
        str2 = "t2_" + self.ip[-3:]
        t2 = threading.Thread(target=self.pub_force, name=str2)
        t2.start()

    def connect(self):
        IP_ADDR = self.ip
        PORT = 4008

        # 创建连接插口并连接
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((IP_ADDR, PORT))

        # 设置采样频率
        set_update_rate = "AT+SMPR=100\r\n"
        s.send(set_update_rate)
        recvData = bytearray(s.recv(1000))
        print recvData

        # 设置矩阵运算单位
        set_compute_unit = "AT+DCPCU=MVPV\r\n"
        s.send(set_compute_unit)
        recvData = bytearray(s.recv(1000))
        print recvData

        # 上传数据格式
        set_recieve_format = "AT+SGDM=(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)\r\n"
        s.send(set_recieve_format)
        recvData = bytearray(s.recv(1000))
        print recvData

        # 连续上传数据包
        get_data_stream = "AT+GSD\r\n"
        s.send(get_data_stream)
        F = np.zeros(6)
        while True:
            data = s.recv(1000)
            fx = struct.unpack("f", data[6:10])[0]
            fy = struct.unpack('f', data[10:14])[0]
            fz = struct.unpack('f', data[14:18])[0]
            mx = struct.unpack('f', data[18:22])[0]
            my = struct.unpack('f', data[22:26])[0]
            mz = struct.unpack('f', data[26:30])[0]
            F[0] = float(fx)
            F[1] = float(fy)
            F[2] = float(fz)
            F[3] = float(mx)
            F[4] = float(my)
            F[5] = float(mz)
            self.F = F

    def pub_force(self):
        # 求取偏置
        F_offset = np.zeros(6)
        for i in range(50):
            F_offset = F_offset + self.F
            time.sleep(0.01)
        F_offset = F_offset / 50.0

        # 发送关节角度
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            F = self.F - F_offset
            command_force = WrenchStamped()
            command_force.wrench.force.x = F[0]
            command_force.wrench.force.y = F[1]
            command_force.wrench.force.z = F[2]
            command_force.wrench.torque.x = F[3]
            command_force.wrench.torque.y = F[4]
            command_force.wrench.torque.z = F[5]
            self.pub.publish(command_force)
            rate.sleep()

def main():
    rospy.init_node('force_sensor_node')
    pub1 = rospy.Publisher("/armr/ft_sensor_topic", WrenchStamped, queue_size=100)
    pub2 = rospy.Publisher("/armt/ft_sensor_topic", WrenchStamped, queue_size=100)
    pub3 = rospy.Publisher("/armc/ft_sensor_topic", WrenchStamped, queue_size=100)

    # #建立传感器
    ip1 = "192.168.0.101"
    sensor1 = TcpSensor(ip1, pub1)

    ip2 = "192.168.0.102"
    sensor2 = TcpSensor(ip2, pub2)

    ip3 = "192.168.0.103"
    sensor3 = TcpSensor(ip3, pub3)

if __name__ == '__main__':
    main()