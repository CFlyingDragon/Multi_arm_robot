#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于s型函数实验
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：2020/8/15
import numpy as np
from math import pi
import math
import numpy.linalg as nla
import sys
import time

import matplotlib.pyplot as plt
from robot_python import MyPlot

def sfunc(x, k=1, a=1000, b=1, m = 0.005):
    y = k/(1 + math.exp(m*(a - b*x)))
    return y

def main():
    num = 10000
    x = np.linspace(0, 2000, num)
    y = np.zeros(num)
    for i in range(num):
        y[i] = sfunc(x[i])

    string2 = 's plot!'
    MyPlot.plot2d(x, y, string2)

if __name__ == "__main__":
    main()