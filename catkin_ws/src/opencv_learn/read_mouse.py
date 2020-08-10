#!/usr/bin/env python
# -*-coding:utf-8-*-
#**read_mouse.py**#
#本文档用于鼠标绘图相关操作
  #程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020/8/3

import cv2 as cv
import numpy as np

events = [i for i in dir(cv) if 'EVENT' in i]
print events

img = np.zeros((512, 512, 3), np.int8)
#鼠标回调函数
def draw_circl(event,x,y,flags,param):
    if events == cv.EVENT_LBUTTONDBLCLK:
        cv.circle(img, (x, y), 100, (255, 0, 0), -1)

cv.namedWindow("image")

#调用鼠标回调函数
cv.setMouseCallback('image', draw_circl)

while True:
    cv.imshow('image', img)
    if cv.waitKey(20) & 0xFF == 27:
        break
cv.destroyAllWindows()