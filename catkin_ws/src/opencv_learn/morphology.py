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
file_name = "pic2.png"
path = os.path.join(file_path, file_name)

#读取图片
img = cv.imread(path, 0)
cv.imshow('img', img)

#卷积内核
kernel = np.ones((5, 5), np.uint8)

#侵蚀
erosion = cv.erode(img, kernel, iterations=1)
cv.imshow('erosion', erosion)

#扩张
dilation = cv.dilate(img, kernel, iterations=1)
cv.imshow('dilation', dilation)

#开运算,侵蚀后扩张
opening = cv.morphologyEx(img, cv.MORPH_OPEN, kernel)
cv.imshow('opening', opening)

#闭运算
closing = cv.morphologyEx(img, cv.MORPH_CLOSE, kernel)
cv.imshow('closing', closing)

#形态学梯度
gradient = cv.morphologyEx(img, cv.MORPH_GRADIENT, kernel)
cv.imshow('gradient', gradient)

#顶帽：输入图像与图像开运算的差值
tophat = cv.morphologyEx(img, cv.MORPH_TOPHAT, kernel)
cv.imshow('tophat', tophat)

#黑帽：闭运算之差
blackhat = cv.morphologyEx(img, cv.MORPH_BLACKHAT, kernel)
cv.imshow('blackhat', blackhat)

cv.waitKey(0)
cv.destroyAllWindows()