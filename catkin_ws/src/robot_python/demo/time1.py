#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于介绍时间
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020年5月26号

#导入时间模块
import time

#获取时间戳
t1 = time.time()
t2 = time.clock()
print t1
print t2

#统计程序执行时间
t1 = time.time()
#程序开始
time.sleep(3)
print "中间程序"
#程序结束
t2 = time.time()
print "运行程序的时间：",t2 - t1