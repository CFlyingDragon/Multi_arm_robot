#!/usr/bin/env python
# -*-coding:utf-8-*-
#**image_smoothing.py**#
#本文档用于图形平滑
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：2020/8/4

import cv2 as cv
import numpy as np

img = cv.imread('opencv.jpg') #+ 0.01*np.random.randint(0, 255, size=(268, 268, 3))

cv.imshow('image1', img)

#二维卷积
kernel = np.ones((5, 5), np.float32)/25
dst = cv.filter2D(img, -1, kernel)
cv.imshow('image2', dst)

#高斯模糊
blur = cv.GaussianBlur(img, (5, 5), 0)
cv.imshow("gauss", img)

#中位模糊
median = cv.medianBlur(img, 5)
cv.imshow("median", median)

#双边滤波
blur = cv.bilateralFilter(img, 9, 75, 75)
cv.imshow('bilateral', blur)

cv.waitKey(0)
cv.destroyAllWindows()
