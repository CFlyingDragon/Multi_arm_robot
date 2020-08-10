#!/usr/bin/env python
# -*-coding:utf-8-*-
#**threshold.py**#
#本文档用于图形几何变换
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：2020/8/4

import cv2 as cv

img = cv.imread('opencv.jpg', 0)
#图像阀值
ret, thresh1 = cv.threshold(img, 127, 255, cv.THRESH_BINARY)
ret, thresh2 = cv.threshold(img, 127, 255, cv.THRESH_BINARY_INV)
ret, thresh3 = cv.threshold(img, 127, 255, cv.THRESH_TRUNC)
ret, thresh4 = cv.threshold(img, 127, 255, cv.THRESH_TOZERO)
ret, thresh5 = cv.threshold(img, 127, 255, cv.THRESH_TOZERO_INV)
cv.imshow('img1', thresh1)
cv.imshow('img2', thresh2)
cv.imshow('img3', thresh3)
cv.imshow('img4', thresh4)
cv.imshow('img5', thresh5)

#Otsu阀值
ret1, th1 = cv.threshold(img, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

#高斯滤波后采用Otsu阀值
blur = cv.GaussianBlur(img, (5, 5), 0)
ret2, th2 = cv.threshold(blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
cv.imshow('th1', th1)
cv.imshow('th2', th2)

cv.waitKey(0)
cv.destroyAllWindows()


