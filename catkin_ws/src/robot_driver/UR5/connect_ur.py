#!/usr/bin/python
#-*-coding:utf-8-*-
import socket
import struct
import time
from math import pi

#连接机器人：采用socket
def connect_robot(ip):
    HOST = ip   
    PORT = 30003			
    robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    robot.settimeout(2.0)
    robot.connect((HOST, PORT))
    return robot

#采用UR的脚本函数,下发关节角度,设置了速度和加速度
def movej(robot,q):
    data ='movej('+str(q)+',a=0.5,v=0.3)\n'
    robot.send(data)

#急停函数
def stopj(robot,a):
    data ='stopj('+str(a)+')\n'
    robot.send(data)

#读取关节角度函数
def read(robot):
    recvData=bytearray(robot.recv(1108))

    # actual joint angles
    q1 = struct.unpack('!d', recvData[252:260])[0]
    q2 = struct.unpack('!d', recvData[260:268])[0]
    q3 = struct.unpack('!d', recvData[268:276])[0]
    q4 = struct.unpack('!d', recvData[276:284])[0]
    q5 = struct.unpack('!d', recvData[284:292])[0]
    q6 = struct.unpack('!d', recvData[292:300])[0]

    # actual joint angle velocity
    qd1 = struct.unpack('!d', recvData[300:308])[0]
    qd2 = struct.unpack('!d', recvData[308:316])[0]
    qd3 = struct.unpack('!d', recvData[316:324])[0]
    qd4 = struct.unpack('!d', recvData[324:332])[0]
    qd5 = struct.unpack('!d', recvData[332:340])[0]
    qd6 = struct.unpack('!d', recvData[340:348])[0]

    # the pose of tcp
    x = struct.unpack('!d', recvData[444:452])[0]
    y = struct.unpack('!d', recvData[452:460])[0]
    z = struct.unpack('!d', recvData[460:468])[0]
    rx = struct.unpack('!d', recvData[468:476])[0]
    ry = struct.unpack('!d', recvData[476:484])[0]
    rz = struct.unpack('!d', recvData[484:492])[0]

    # the velocity of tcp
    vx = struct.unpack('!d', recvData[492:500])[0]
    vy = struct.unpack('!d', recvData[500:508])[0]
    vz = struct.unpack('!d', recvData[508:516])[0]
    v_rx = struct.unpack('!d', recvData[516:524])[0]
    v_ry = struct.unpack('!d', recvData[524:532])[0]
    v_rz = struct.unpack('!d', recvData[532:540])[0]

    # the force of tcp
    Fx = struct.unpack('!d', recvData[540:548])[0]
    Fy = struct.unpack('!d', recvData[548:556])[0]
    Fz = struct.unpack('!d', recvData[556:564])[0]
    Tx = struct.unpack('!d', recvData[564:572])[0]
    Ty = struct.unpack('!d', recvData[572:580])[0]
    Tz = struct.unpack('!d', recvData[580:588])[0]

    #print([q1*180/pi,q2*180/pi,q3*180/pi,q4*180/pi,q5*180/pi,q6*180/pi])
    print([q1,q2,q3,q4,q5,q6])

    print([qd1,qd2,qd3,qd4,qd5,qd6])

    print([x*1000,y*1000,z*1000,rx,ry,rz])

    print([vx,vy,vz,v_rx,v_ry,v_rz])

    print([Fx,Fy,Fz,Tx,Ty,Tz])


def read_joints(robot):

    recvData=bytearray(robot.recv(1108))

    q1 = struct.unpack('!d', recvData[252:260])[0]
    q2 = struct.unpack('!d', recvData[260:268])[0]
    q3 = struct.unpack('!d', recvData[268:276])[0]
    q4 = struct.unpack('!d', recvData[276:284])[0]
    q5 = struct.unpack('!d', recvData[284:292])[0]
    q6 = struct.unpack('!d', recvData[292:300])[0]

    return [q1,q2,q3,q4,q5,q6]


def main():
    #robot1 = connect_robot('192.168.0.1')
    robot2 = connect_robot('192.168.0.2')
    #robot3 = connect_robot('192.168.0.3')
    #stopj(robot1, 0.5)
    stopj(robot2, 0.5)
    #stopj(robot3, 0.5)

if __name__=="__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

