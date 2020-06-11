#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于传感器通讯
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.6.8
#tcp协议通信
import socket
import time

#设置ip和端口
ip = '192.168.0.2'
port = 4008

#创建一个socket
mysocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#连接
mysocket.connect((ip, port))

#接收数据
recvDate = mysocket.recv(1024)
print recvDate

#关闭连接
mysocket.close()