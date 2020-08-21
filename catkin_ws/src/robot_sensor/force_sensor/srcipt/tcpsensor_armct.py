#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于传感器通,加入ros节点发送消息
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2020.6.13
#系统函数
#传感求编号SN 04906
import numpy as np
import threading
import time

import struct
import socket

import rospy
from geometry_msgs.msg import WrenchStamped

#设置全局变量
F = np.zeros(6)
def tcp_connect():
    '''
        采集卡：M8128B1
              SN04906
        传感器：
        M3705C
        SN8959
        :return:
        '''
    #设置IP和端口号
    IP_ADDR	= '192.168.0.108'
    PORT = 4008

    #创建连接插口并连接
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((IP_ADDR, PORT))

    #设置传感器参数
    #设置解耦矩阵,仅需要设置一次
    # decouple_matrix = "(0.74447,-0.10432,-4.05718,113.20804,-0.19615,-112.78654);" \
    #                       "(-1.55915,-131.20568,-12.28236,63.88653,2.47576,65.56931);" \
    #                       "(320.82506,-6.00990,322.04951,1.83496,324.31629,-3.44262);" \
    #                       "(0.13401,0.15065,-11.66469,-0.15551,11.42006,-0.23770);" \
    #                       "(13.19140,-0.21842,-6.52051,0.03898,-6.59893,0.00286);" \
    #                       "(-0.01730,4.59975,-0.19084,4.56898,0.06837,4.54050)\r\n";
    # set_decouple_matrix = "AT+DCPM=" + decouple_matrix
    # s.send(set_decouple_matrix)
    # recvData = bytearray(s.recv(1000))
    # print "新设置的解耦矩阵：", recvData

    #设置采样频率
    set_update_rate = "AT+SMPR=100\r\n"
    s.send(set_update_rate)
    recvData = bytearray(s.recv(1000))
    print recvData

    #设置矩阵运算单位
    set_compute_unit = "AT+DCPCU=MVPV\r\n"
    s.send(set_compute_unit)
    recvData = bytearray(s.recv(1000))
    print recvData

    #上传数据格式
    set_recieve_format= "AT+SGDM=(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)\r\n"
    s.send(set_recieve_format)
    recvData = bytearray(s.recv(1000))
    print recvData

    #连续上传数据包
    get_data_stream = "AT+GSD\r\n"
    s.send(get_data_stream)
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


def pub_force_node():
    # 运行话题
    rospy.init_node('force_sensor_armt_node')
    pub = rospy.Publisher("/armt/ft_sensor_topic", WrenchStamped, queue_size=100)

    #建立线程接收六维力
    t1 = threading.Thread(target=tcp_connect)
    t1.start()
    time.sleep(2)

    #求取偏置
    F_offset = np.zeros(6)
    for i in range(10):
        F_offset = F_offset + F
        time.sleep(0.01)
    F_offset = F_offset/10.0

    # 发送关节角度
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        F1 = F - F_offset
        command_force = WrenchStamped()
        command_force.wrench.force.x = F1[0]
        command_force.wrench.force.y = F1[1]
        command_force.wrench.force.z = F1[2]
        command_force.wrench.torque.x = F1[3]
        command_force.wrench.torque.y = F1[4]
        command_force.wrench.torque.z = F1[5]
        pub.publish(command_force)
        #if(k/10==0):
        print "六维力：",np.around(F1, 3)
        rate.sleep()

if __name__ == '__main__':
    pub_force_node()