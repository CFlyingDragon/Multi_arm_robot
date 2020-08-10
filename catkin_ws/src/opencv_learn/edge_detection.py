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

#加载图片
img = cv.imread(path, 0)
cv.imshow('img', img)

#canny边缘检测
edges = cv.Canny(img, 100, 200)
cv.imshow('edges', edges)

cv.waitKey(0)
cv.destroyAllWindows()