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
import matplotlib.pyplot as plt
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

#======================高增益观测器:示教用途=======================#
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

#======================数据处理=======================#
def deal_with_data1():
    #获取地址
    file_path = os.path.join(os.getcwd(), '../../', 'data/impedance/sigle_z')
    file_path = os.path.abspath(file_path)
    file_pos = file_path + '/position1.txt'
    print file_path
    file_force = file_path + '/force1.txt'

    #读取数据
    pos = FileOpen.read(file_pos)
    force = FileOpen.read(file_force)
    num1 = len(pos)
    print "num1:", num1
    T = 0.01
    t1 = np.linspace(0, T*(num1-1), num1)
    num2 = len(force)
    print "num2:", num2
    t2 = np.linspace(0, T * (num2 - 1), num2)
    #绘制数据图
    MyPlot.plot2_nd(t1, pos, title='position', xlab='t/s', ylab='red', lable='qq')
    MyPlot.plot2_nd(t2, force, title='force', xlab='t/s', ylab='N', lable='f')

    #求取正运动学
    DH0 = rp.DHf_armt
    XX = np.zeros([num1, 6])
    for i in range(num1):
        XX[i, :] = kin.fkine_euler(DH0, pos[i, :])

    #绘制单轴曲线
    plt.figure(2)
    plt.plot(t1[5000:12000], XX[5000:12000, 2], label='Xz', color='r')
    plt.title("Xz")
    plt.xlabel("t/s")
    plt.ylabel("x/m")
    plt.legend()

    plt.figure(3)
    plt.plot(t2[6000:14000], force[6000:14000, 2], label='Fz', color='r')
    plt.title("Fz")
    plt.xlabel("t/s")
    plt.ylabel("fz/N")
    plt.legend()

    plt.show()

    #写入文件
    # 获取地址
    file_path = os.path.join(os.getcwd(), '../../', 'data/impedance/deal_with')
    file_path = os.path.abspath(file_path)
    file_pos = file_path + '/armt_real_Xz1.txt'
    file_force = file_path + '/armt_real_Fz1.txt'
    FileOpen.write([XX[5000:12000, 2]], file_pos)
    FileOpen.write([force[6000:14000, 2]], file_force)

def deal_with_data2():
    # 获取地址
    file_path = os.path.join(os.getcwd(), '../../', 'data/impedance/deal_with')
    file_path = os.path.abspath(file_path)
    file_pos = file_path + '/armt_real_Xz1.txt'
    file_force = file_path + '/armt_real_Fz1.txt'

    #读取数据
    pos = FileOpen.read(file_pos)[-1, :]
    force = FileOpen.read(file_force)[-1, :]
    num1 = len(pos)
    print "num1:", num1
    T = 0.01
    t1 = np.linspace(0, T*(num1-1), num1)
    num2 = len(force)
    print "num2:", num2
    t2 = np.linspace(0, T * (num2 - 1), num2)

    #绘制单轴曲线
    plt.figure(1)
    plt.plot(t1, pos, label='Xz', color='r')
    plt.title("Position: Xz")
    plt.xlabel("t/s")
    plt.ylabel("x/m")
    plt.legend()

    plt.figure(2)
    plt.plot(t2, force, label='Fz', color='r')
    plt.title("Force: Fz")
    plt.xlabel("t/s")
    plt.ylabel("fz/N")
    plt.legend()



    #截取每段中相对平稳的部分:每段截取100个点
    idx_pos = [[0, 400],
               [600, 1000],
               [1300, 1700],
               [1850, 2250],
               [2500, 2900],
               [3300, 3700],
               [4000, 4400],
               [4550, 4950],
               [5050, 5450],
               [5600, 6000],
               [6400, 6800]]
    idx_force = [[0, 400],
                 [600, 1000],
                 [1400, 1800],
                 [2000, 2400],
                 [2800, 3200],
                 [3700, 4100],
                 [4500, 4900],
                 [5200, 5600],
                 [5750, 6150],
                 [6400, 6800],
                 [7300, 7700]]
    #求取刚度
    m = 11
    k = np.zeros(m-1)
    f = np.zeros(m)
    ff = np.zeros(m-1)
    x = np.zeros(m)

    for i in range(m):
        f[i] = sum(force[idx_force[i][0]:idx_force[i][1]])/400
        x[i] = sum(pos[idx_pos[i][0]:idx_pos[i][1]])/400
    for i in range(m - 1):
        k[i] = (f[i]-f[i+1])/(x[i]-x[i+1])
        ff[i] = (f[i] + f[i+1])/2

    '''
    基础位置qq[0, 60, 0, 60, 0, 60, 0]
    所以z轴力在基座标系或在工具坐标系中只是方向相反
    '''
    x = x*1000 #单位改为mm
    print "k: ", np.round(k, 3)
    print "f: ", np.around(f, 3)
    print "x: ", np.around(x, 3)

    # 绘制单轴曲线
    plt.figure(3)
    plt.plot(f[:7], x[0:7], label='increase', color='r',
             linestyle=':', marker='o', markerfacecolor='r', markersize=8)
    plt.plot(f[6:], x[6:], label='reduce', color='b',
             linestyle=':', marker='x', markerfacecolor='b', markersize=8)
    plt.title("Position and force diagram")
    plt.xlabel("Fz/N")
    plt.ylabel("Xz/mm")
    plt.legend()

    plt.figure(4)
    plt.plot(ff[:5], k[:5], label='increase', color='r',
             linestyle=':', marker='o', markerfacecolor='r', markersize=8)
    plt.plot(ff[5:], k[5:], label='reduce', color='b',
             linestyle=':', marker='x', markerfacecolor='b', markersize=8)
    plt.title("Relationship between stiffness and force")
    plt.xlabel("Fz/N")
    plt.ylabel("Kz/(N/m)")
    plt.legend()
    plt.show()


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
    #main()
    deal_with_data2()
    print "finish!"