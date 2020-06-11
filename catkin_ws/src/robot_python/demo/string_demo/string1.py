#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于字符串处理
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2020.6.9
#系统函数
import string

#建立字符串
str1 = "My first string"
print type(str1)

#其他类型转换为字符串
list_ = [1, 2, 3, 4]
str2 = str(list_)
print type(list_), "转换为", type(str2)

#直接读取字符串值
for i in str1:
    print i

#下标索引字符串
l = len(str2)
for i in range(l):
    print "第", i, "个字符串：", str2[i]

#切片处理
str3 = str1[3:6]
print "第4,5,6字符为：",str3

#字符串拼接处理
str4 = str1 + str2
print "拼接后的字符串：", str4

#===========string函数介绍========#
