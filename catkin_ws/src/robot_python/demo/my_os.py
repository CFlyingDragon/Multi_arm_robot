#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于学习os
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020年6月20日

#导入os
import os

#获取当前路径
path = os.getcwd()
print path

path = os.path.dirname(__file__)
print path

path = os.path.abspath(os.path.dirname(__file__))
print path

#获得上级目录
path = os.path.join(os.getcwd(), "..")
print path
path = os.path.abspath(os.path.join(os.getcwd(), ".."))
print path
path = os.path.abspath(os.path.join(os.getcwd(), "../.."))
print path

#分离路径
path = os.path.split(path)
print path
path = os.path.join(os.getcwd(),"name.txt")
path = os.path.splitext("name.txt")
print path

path = os.path.dirname("demo")
print path