#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于scipy学习
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020年5月30号

import numpy as np
import matplotlib.pyplot as plt

#导入插值模块
from scipy.interpolate import interp2d

#生成数据
x = np.linspace(0, 1, 20)
y = np.linspace(0, 1, 30)
xx, yy = np.meshgrid(x, y)
rand = np.random.rand(600).reshape([30, 20])
z = np.sin(xx**2) + np.cos(yy**2) + rand

new_x = np.linspace(0, 1, 100)
new_y = np.linspace(0, 1, 100)

#**样条插值函数插值函数***#
#一次插值
z1 = interp2d(x, y, z, kind='linear')
new_z1 = z1(new_x, new_y)

#三次插值
z3 = interp2d(x, y, z, kind='cubic')
new_z3 = z3(new_x, new_y)

#绘图
plt.figure()
plt.plot(x, z[0, :], 'o', label='data')
plt.plot(new_x, new_z1[0, :], label='linear')
plt.plot(new_x, new_z3[0, :], label='cubic')
plt.title("interp2d")
plt.xlabel("x")
plt.ylabel("f")
plt.legend()
plt.show()

#用矩阵显示z
plt.matshow(z)
plt.title("data")
plt.xlabel("x")
plt.ylabel("y")
plt.show()

#用矩阵显示z
plt.matshow(new_z1)
plt.title("linear")
plt.xlabel("x")
plt.ylabel("y")
plt.show()

#用矩阵显示z
plt.matshow(new_z3)
plt.title("cubic")
plt.xlabel("x")
plt.ylabel("y")
plt.show()
