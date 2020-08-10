#!/usr/bin/env python
# -*-coding:utf-8-*-
# **plot_trajectory.py**#
# 本文档用于轨迹栏作为调色板
# 程序员：陈永厅
# 版权：哈尔滨工业大学
# 日期：初稿：2020/8/3

import numpy as np
import cv2 as cv

#接收相机数据
cap = cv.VideoCapture(0)

while(1):
    #读取帧
    _, frame = cap.read()
    #转换颜色空间 BGR到HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    #定义HSV中蓝色的范围
    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])
    #设置HSV的阀值使得只得去蓝色
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    #将淹模逐像素相加
    res = cv.bitwise_and(frame, frame, mask=mask)
    cv.imshow('frame', frame)
    cv.imshow('mask', mask)
    cv.imshow('res', res)
    k = cv.waitKey(5) & 0xFF
    if k==27:
        break
cv.destroyAllWindows()
