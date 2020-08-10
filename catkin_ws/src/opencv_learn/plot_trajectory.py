#!/usr/bin/env python
# -*-coding:utf-8-*-
# **plot_trajectory.py**#
# 本文档用于轨迹栏作为调色板
# 程序员：陈永厅
# 版权：哈尔滨工业大学
# 日期：初稿：2020/8/3

import numpy as np
import cv2 as cv

def nothing(x):
    pass

#创建一个黑色的图像,一个窗口
img = np.zeros((300, 512, 3), np.uint8)
cv.namedWindow('image')

#创建颜色变换轨迹
cv.createTrackbar('R', 'image', 0, 255, nothing)
cv.createTrackbar('G', 'image', 0, 255, nothing)
cv.createTrackbar('B', 'image', 0, 255, nothing)

#为ON/OFF功能创建开关
switch = '0:OFF \n1 : ON'
cv.createTrackbar(switch, 'image', 0, 1, nothing)

while(1):
    cv.imshow('image', img)
    k = cv.waitKey(1) & 0xFF
    if k==27:
        break

    #得到四条轨迹线的当前位置
    r = cv.getTrackbarPos('R', 'image')
    g = cv.getTrackbarPos('G', 'image')
    b = cv.getTrackbarPos('B', 'image')
    s = cv.getTrackbarPos(switch, 'image')

    if s == 0:
        img[:] = 0
    else:
        img[:] = [b, g, r]

cv.destroyAllWindows()
