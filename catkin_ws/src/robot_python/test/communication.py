# usr/bin/python
# -*- coding: utf-8 -*-

#导入串口模块
import serial

#打开串口
ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.5)
print ser

#关闭端口
ser.close()
print 1
#打开端口
ser.open()

#写入变量
data1 = [0x41, 0x54, 0x2B, 0x47, 0x4F, 0x44, 0x0D, 0x0A]
ser.write(data1)
print 2
#读取10个字节数据
data = ser.readline()
print data


