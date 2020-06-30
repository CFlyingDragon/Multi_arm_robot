#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于速度评估
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020年6月29号
import numpy as np
import matplotlib.pyplot as plt
from numpy import random

from robot_python import MyPlot

#导入插值模块
from scipy.interpolate import interp1d

def observer(qq_list, qv_list, alpha1 = 400, alpha2 = 20, T = 0.01):
    a1 = 4 / alpha2 + 2 * alpha1 / alpha2 + 1
    a2 = -8 / alpha2 + 2
    a3 = -4 / alpha2 - 2 * alpha1 / alpha2 + 1

    qv = (2/T*(qq_list[2] - qq_list[0]) - a2*qv_list[1] - a3*qv_list[0])/a1
    return qv

def high_gain_observer():
    num = 1000
    T = 0.01
    t = np.linspace(0, T*(num - 1), num)

    qq = 5*np.cos(np.pi/2 * t) + 0.5*random.random(num)
    qv = -5*np.sin(np.pi/2 * t)*np.pi/2

    qq_list = np.zeros(3)
    qq_list[0] = qq[0]
    qq_list[1] = qq[0]
    qq_list[2] = qq[0]
    qv_list = np.zeros(2)

    qv_o = np.zeros(num)

    for i in range(num):
        qq_list[0] = qq_list[1]
        qq_list[1] = qq_list[2]
        qq_list[2] = qq[i]
        qv_o[i] = observer(qq_list, qv_list)

        qv_list[0] = qv_list[1]
        qv_list[1] = qv_o[i]

    qv_d = np.zeros(num)
    for i in range(num):
        if i==0:
            qv_d[i] = 0
        else:
            qv_d[i] = (qq[i] - qq[i - 1])/T

    Qv = np.zeros([num, 3])
    Qv[:, 0] = qq
    Qv[:, 1] = 0.6  * qv_o
    Qv[:, 2] = qv
    MyPlot.plot2_nd(t, Qv, title="Qv")
    return Qv

def smooth(Qv):
    WINDOWS = ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']
    def smooth(x, window_len=11, window='hanning'):
        if x.ndim != 1:
            raise ValueError('smooth only accepts 1 dimension arrays.')
        if x.size < window_len:
            raise ValueError('Input vector needs to be bigger than window size.')
        if window_len < 3:
            return x
        if not window in WINDOWS:
            raise ValueError('Window is one of "flat", "hanning", "hamming", "bartlett", "blackman"')
        s = np.r_[x[window_len - 1:0:-1], x, x[-1:-window_len:-1]]
        if window == 'flat':
            w = np.ones(window_len, 'd')
        else:
            w = eval('np.' + window + '(window_len)')
        y = np.convolve(w / w.sum(), s, mode='valid')
        return y

    num = len(Qv)
    T = 0.01
    t = np.linspace(0, T*(num -1), num)

    y = smooth(Qv[:, 1])
    x = Qv[:, 2]
    xn = Qv[:, 1]
    ws = 31

    plt.figure()

    plt.subplot(211)
    plt.plot(np.ones(ws))
    for w in WINDOWS[1:]:
        eval('plt.plot(np.' + w + '(ws))')
    plt.axis([0, 30, 0, 1.1])
    plt.legend(WINDOWS)
    plt.title('Smoothing windows')

    plt.subplot(212)
    plt.plot(x)
    plt.plot(xn)
    for w in WINDOWS:
        plt.plot(smooth(xn, 10, w))
    l = ['original signal', 'signal with noise']
    l.extend(WINDOWS)
    plt.legend(l)
    plt.title('Smoothed signal')

    plt.show()

if __name__ == "__main__":
    Qv = high_gain_observer()
    smooth(Qv)