#!/usr/bin/env python
# -*-coding:utf-8-*-
#**read_video.py**#
#本文档用于读取相机的视屏
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020/8/3

import numpy as np
import cv2 as cv
import os

file_path = os.path.abspath(os.path.join(os.getcwd(), "./", "data"))
file_name = "my_video.avi"
path = os.path.join(file_path, file_name)

#加载摄像头:选择摄像头0、1,加载文件：给定文件地址
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print "摄像头打开失败！"
    exit()

#定义编解码器并创建ViodeoWriter对象
fourcc = cv.VideoWriter_fourcc(*'XVID')
out = cv.VideoWriter(path, fourcc, 20.0, (640, 480))

while cap.isOpened():
    #逐帧捕捉
    ret, frame = cap.read()
    #如果正确读取帧，ret为True
    if not ret:
        print "读取帧失败！"
        break
        
    #反转图像
    frame = cv.flip(frame, 0)

    #写入文件
    out.write(frame)

    #显示处理结果
    cv.imshow("frame", frame)

    #退出操作
    if cv.waitKey(1) == ord('q'):
        break

#完成所有操作后，释放捕捉器
cap.release()
out.release()
cv.destroyAllWindows()
