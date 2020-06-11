#-*-coding:utf-8-*-
#!/usr/bin/env python
#本文档用于绘制关节反馈数据离线图
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020年6月1号

import numpy as np
from math import pi
import matplotlib.pyplot as plt
from sub_write_joint_states import path

from robot_python import FileOpen

def joint_plot():

    #读取关节角度
    qq = np.array(FileOpen.read(path))

    #接收时间
    T = 0.01
    [k, n] = qq.shape

    #生成时间
    t = np.linspace(0, (k-1)*T, k)

    #绘制图版
    plt.figure()

    #绘制每个关节角
    for i in range(n-1):
        qi_str = "q" + str(i + 1)
        plt.plot(t, qq[:, i+1], label=qi_str)

    plt.title('Joints position plot!')
    plt.xlabel("t/s")
    plt.ylabel("position/rad")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    joint_plot()

