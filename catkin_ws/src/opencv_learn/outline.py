#!/usr/bin/env python
# -*-coding:utf-8-*-
#**read_video.py**#
#本文档用于读取相机的视屏
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020/8/5

import numpy as np
import cv2 as cv
import os

file_path = os.path.abspath(os.path.join(os.getcwd(), "./", "data"))
file_name = "pic3.png"
path = os.path.join(file_path, file_name)

#加载图片
img = cv.imread(path)
cv.imshow('img', img)

# #变成灰度图
imgray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imshow('gray', imgray)

#阀值处理
ret, binary = cv.threshold(imgray, 127, 255, cv.THRESH_BINARY)
cv.imshow('binary', binary)

#获取轮廓
'''
第一个，它返回了你所处理的图像
第二个，正是我们要找的，轮廓的点集
第三个，各层轮廓的索引
'''
_, contours, hierarchy = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

#绘制轮廓
cv.drawContours(img, contours, -1, (0, 0, 255), 3)
cv.imshow('contours', img)

#特征矩
cnt = contours[0]
M = cv.moments(cnt)
print M

#质心
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])
print "质心：", [cx, cy]

#轮廓面积
area = cv.contourArea(cnt)
print "面积： ", area

#周长
perimeter = cv.arcLength(cnt, True)
print '周长： ', perimeter

#轮廓近似
epsilon = 0.1*cv.arcLength(cnt, True)
approx = cv.approxPolyDP(cnt, epsilon, True)
print "简化轮廓: ", approx.shape
cv.drawContours(img, approx, -1, (0, 0, 255), 1)
cv.imshow('approx', img)

#检查凸度
k = cv.isContourConvex(cnt)
print 'k: ', k

#直角矩形：用未旋转的矩形包围物体
x, y, w, h = cv.boundingRect(cnt)
cv.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
cv.imshow('rect1', img)

#旋转矩形：用旋转过的矩形包围物体，使面积最小
rect = cv.minAreaRect(cnt) #返回Box2D结构包含了中心、（宽,高）,旋转角度
box = cv.boxPoints(rect)
box = np.int0(box)
cv.drawContours(img, [box], 0, (255, 0, 0), 2)
cv.imshow('rect2', img)

#最小闭合圆：用圆包围
(x, y), radius = cv.minEnclosingCircle(cnt)
center = (int(x), int(y))
radius = int(radius)
cv.circle(img, center, radius, (0, 255, 0), 2)
cv.imshow('circle', img)

#拟合一个椭圆
ellipse = cv.fitEllipse(cnt)
cv.ellipse(img, ellipse, (0, 255, 255), 2)
cv.imshow('ellipse', img)

cv.waitKey(0)