#!/usr/bin/env python
# -*-coding:utf-8-*-
# **plot_graphical.py**#
# 本文档用于绘制图形
# 程序员：陈永厅
# 版权：哈尔滨工业大学
# 日期：初稿：2020/8/3

import numpy as np
import cv2 as cv
import os

file_path = os.path.abspath(os.path.join(os.getcwd(), "./", "data"))
file_name = "my_video.avi"
path = os.path.join(file_path, file_name)

##************绘制直线**************##
#创建黑色的图像
img = np.zeros((512, 512, 3), np.uint8)
#绘制一条厚度为5的蓝色对角线
cv.line(img, (0, 0), (511, 511), (255, 0, 0), 5)
#显示图像
cv.imshow("line", img)
cv.waitKey(0)

##************绘制矩阵**************##
#绘制一条厚度为5的蓝色对角线
cv.rectangle(img, (384, 0), (510, 128), (0, 255, 0), 3)
#显示图像
cv.imshow("rectangle", img)
cv.waitKey(0)

##************绘制圆形**************##
#绘制一条厚度为5的蓝色对角线
cv.circle(img, (447, 63), 63, (0, 255, 0), -1)
#显示图像
cv.imshow("circle", img)
cv.waitKey(0)

##************绘制圆形**************##
#绘制圆形
cv.circle(img, (447, 63), 63, (0, 0, 255), -1)
#显示图像
cv.imshow("circle", img)
cv.waitKey(0)

##************绘制椭圆**************##
#绘制椭圆
cv.ellipse(img, (256, 256), (100, 50), 0, 0, 180, 255, -1)
#显示图像
cv.imshow("ellipse", img)
cv.waitKey(0)

##************绘制多边形**************##
pts = np.array([[10, 0], [20, 20], [70, 20], [50, 10]], np.int32)
pts = pts.reshape((-1, 1, 2))
#绘制多边形
cv.polylines(img, [pts], True, (0, 255, 255))
#显示图像
cv.imshow("polylines", img)
cv.waitKey(0)

##************添加文字**************##
#文字类型
font = cv.FONT_HERSHEY_SIMPLEX
#添加文字
cv.putText(img, "OpenCV", (10, 500), font, 4, (255, 255, 255), 2, cv.LINE_AA)
#显示图像
cv.imshow("TEXIT", img)
cv.waitKey(0)