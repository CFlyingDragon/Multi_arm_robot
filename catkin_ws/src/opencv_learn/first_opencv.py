#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于测试opencv
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020/8/3

import cv2 as cv
import os

file_path = os.path.abspath(os.path.join(os.getcwd(), "./", "data"))
file_name = "first_picture.png"
path = os.path.join(file_path, file_name)

#加载灰色图片
img = cv.imread(path, 0)

#创建一个窗口，非必要
cv.namedWindow('image', cv.WINDOW_NORMAL)

#显示图片
cv.imshow("image", img)

#保存图片
cv.imwrite("image1.jpg", img)

#延时,设置为0,无限期延时，等待键盘反应
cv.waitKey(0)

#关闭窗口
cv.destroyAllWindows()