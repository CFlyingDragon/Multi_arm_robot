#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于数据处理
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2020.6.30

import os
import numpy as np
import numpy.random as random
import numpy.linalg as nla
import math

import Kinematics as kin
import BaseFunction as bf
import FileOpen
import MachineLearning as ml
import RobotParameter as rp
import MyPlot

#======================数据平滑=======================#
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

#======================高增益观测器=======================#
class HighGainObserver(object):
    def __init__(self):
        pass
    def observer(self, qq_list, qv_list, alpha1 = 400, alpha2 = 20, T = 0.01):
        a1 = 4 / alpha2 + 2 * alpha1 / alpha2 + 1
        a2 = -8 / alpha2 + 2
        a3 = -4 / alpha2 - 2 * alpha1 / alpha2 + 1

        qv = (2/T*(qq_list[2] - qq_list[0]) - a2*qv_list[1] - a3*qv_list[0])/a1
        return qv

    def high_gain_observer(self, qq):
        qq_list = np.zeros(3)
        qq_list[0] = qq[0]
        qq_list[1] = qq[0]
        qq_list[2] = qq[0]
        qv_list = np.zeros(2)

        qv_o = np.zeros(self.num)

        for i in range(self.num):
            qq_list[0] = qq_list[1]
            qq_list[1] = qq_list[2]
            qq_list[2] = qq[i]
            qv_o[i] = self.observer(qq_list, qv_list)

            qv_list[0] = qv_list[1]
            qv_list[1] = qv_o[i]

        qv_d = np.zeros(self.num)
        for i in range(self.num):
            if i == 0:
                qv_d[i] = 0
            else:
                qv_d[i] = (qq[i] - qq[i - 1]) / self.T
        qv_o = 0.6*qv_o
        return [qv_o, qv_d]

    def get_original_data(self, qq, T = 0.01):
        self.qq = np.array(qq)
        self.num = len(qq)
        self.T = T
        self.process_data()

    def process_data(self):
        #平滑原始数据
        qq_process = smooth(self.qq)

        #高增益观测器
        [qv_o, qv_d] = self.high_gain_observer(qq_process)

        #平滑所得数据
        self.qv_o = smooth(qv_o)[-self.num:]
        self.qv_d = smooth(qv_d)[-self.num:]

    def put_observer_data(self):
        return self.qv_o

    def put_diff_data(self):
        return self.qv_d

def main():
    num = 1000
    T = 0.01
    t = np.linspace(0, T * (num - 1), num)

    qq = 5 * np.cos(np.pi / 2 * t) + 0.5 * random.random(num)
    qv = -5 * np.sin(np.pi / 2 * t) * np.pi / 2

    observer = HighGainObserver()
    observer.get_original_data(qq)
    Qv = np.zeros([num, 3])
    Qv[:, 0] = observer.put_observer_data()
    Qv[:, 1] = observer.put_diff_data()
    Qv[:, 2] = qv
    MyPlot.plot2_nd(t, Qv, title="Qv")

if __name__ == "__main__":
    main()