#!/usr/bin/env python
import roslib; roslib.load_manifest('ur_driver')
import time, sys, threading, math
import copy
import datetime
import socket, select
import struct
import traceback, code
import SocketServer

import rospy

from sensor_msgs.msg import JointState
from ur_driver.deserializeRT import RobotStateRT
from ur_msgs.msg import *


joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
joint_offsets = {}   #关节偏执，字典


def __on_packet(buf):
    #解码
    stateRT = RobotStateRT.unpack(buf)

    #填写数据结构,用于实时交互
    msg = RobotStateRTMsg()
    msg.time = stateRT.time
    msg.q_target = stateRT.q_target
    msg.qd_target = stateRT.qd_target
    msg.qdd_target = stateRT.qdd_target
    msg.i_target = stateRT.i_target
    msg.m_target = stateRT.m_target
    msg.q_actual = stateRT.q_actual
    msg.qd_actual = stateRT.qd_actual
    msg.i_actual = stateRT.i_actual
    msg.tool_acc_values = stateRT.tool_acc_values
    msg.tcp_force = stateRT.tcp_force
    msg.tool_vector = stateRT.tool_vector
    msg.tcp_speed = stateRT.tcp_speed
    msg.digital_input_bits = stateRT.digital_input_bits
    msg.motor_temperatures = stateRT.motor_temperatures
    msg.controller_timer = stateRT.controller_timer
    msg.test_value = stateRT.test_value
    msg.robot_mode = stateRT.robot_mode
    msg.joint_modes = stateRT.joint_modes

    #发送机器人的实时状态
    pub_robot_stateRT.publish(msg)
    
    #建立关节角
    msg = JointState()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "From real-time state data"
    msg.name = joint_names
    msg.position = [0.0] * 6              #列表处理方法
    for i, q in enumerate(stateRT.q_actual):
        msg.position[i] = q + joint_offsets.get(joint_names[i], 0.0)
    msg.velocity = stateRT.qd_actual
    msg.effort = [0]*6
    #发送实时位置和速度
    pub_joint_statesRT.publish(msg)

def main():
    rospy.init_node('testRT_comm', disable_signals=True)
    
    global pub_joint_statesRT
    pub_joint_statesRT = rospy.Publisher('joint_statesRT', JointState, queue_size=1)
    global pub_robot_stateRT
    pub_robot_stateRT = rospy.Publisher('robot_stateRT', RobotStateRTMsg, queue_size=1)
    
    
    #*************底层接口************#
    #ip地址和端口号
    robot_hostname = '192.168.0.42'
    rt_port = 30003

    #创建cocket连接
    rt_socket = socket.create_connection((robot_hostname, rt_port))
    buf = ""
    
    while not rospy.is_shutdown():
        #接受小于4096个字节的数据
        more = rt_socket.recv(4096)
        if more:
            buf = buf + more

            #提取数据包
            # Attempts to extract a packet
            packet_length = struct.unpack_from("!i", buf)[0]
            print("PacketLength: ", packet_length, "; BufferSize: ", len(buf))
            if len(buf) >= packet_length:
                #将字符分隔为两段
                packet, buf = buf[:packet_length], buf[packet_length:]
                __on_packet(packet)
        else:
            print("There is no more...")

    #关闭接口
    rt_socket.close()




if __name__ == '__main__': main()
