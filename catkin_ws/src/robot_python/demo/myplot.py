#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于数据处理
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2020.6.30
#如遇中文显示问题可加入以下代码
import numpy as np

import matplotlib.pyplot as plt
#解决中文显示问题
plt.rcParams['font.sans-serif'] = ['SimHei'] # 指定默认字体
plt.rcParams['axes.unicode_minus'] = False # 解决保存图像是负号'-'显示为方块的问题

t = np.linspace(0, 1, 1000)
f = np.sin(np.pi*t)
plt.figure(1)
plt.plot(t, f, label='Fz', color='r')
plt.title(u"未知曲面力跟踪")
plt.xlabel("t(s)")
plt.ylabel("Fz(N)")
plt.legend()
plt.rcParams['savefig.dpi'] = 500  # 图片像素
plt.show()