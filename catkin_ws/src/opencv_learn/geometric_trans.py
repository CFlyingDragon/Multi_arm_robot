#!/usr/bin/env python
# -*-coding:utf-8-*-
#**geometric_trans.py**#
#本文档用于图形几何变换
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：2020/8/4

import numpy as np
import cv2 as cv

img = cv.imread('opencv.jpg',0)

#平移
rows, cols = img.shape
M = np.float32([[1, 0, 100], [0, 1, 50]])
dst = cv.warpAffine(img, M, (cols, rows))
cv.imshow('img1', dst)
cv.waitKey(0)

#旋转
M = cv.getRotationMatrix2D(((cols-1)/2.0,(rows-1)/2.0), 90, 1)
dst = cv.warpAffine(img, M, (cols, rows))
cv.imshow('img2', dst)
cv.waitKey(0)

#仿射变换
img = cv.imread('image1.jpg')
rows, cols, ch = img.shape
pts1 = np.float32([[50, 50], [200, 50], [50, 200]])
pts2 = np.float32([[10, 100], [200, 50], [100, 250]])
M = cv.getAffineTransform(pts1, pts2)
dst = cv.warpAffine(img, M, (cols, rows))
cv.imshow('img', img)
cv.imshow('dst', dst)
cv.waitKey(0)

#透视变换
pts1 = np.float32([[56, 65], [368, 52], [28, 387], [389, 390]])
pts2 = np.float32([[0, 0], [300, 0], [0, 300], [300, 300]])
M = cv.getPerspectiveTransform(pts1, pts2)

dst = cv.warpPerspective(img, M, (300, 300))
cv.imshow('dst1', dst)
cv.waitKey(0)

cv.destroyAllWindows()
