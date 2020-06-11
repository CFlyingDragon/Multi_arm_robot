#!/usr/bin/python
#-*-coding:utf-8-*-
import numpy as np

a = "1, 2, 3, 40.0\x000"
a1 = a.strip()
print "a1:", a1
a2 = a1.split(',')
print "a2:", a2
a3 = np.array(a2)
print "a3:", a3
#运行这一步
b = a3[-1]
b = b[0:-2]
a3[-1] = b
print "处理后的a3:", a3