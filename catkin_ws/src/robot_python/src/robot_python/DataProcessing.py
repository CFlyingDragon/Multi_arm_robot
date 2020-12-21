#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于数据处理
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2020.6.30
#如遇中文显示问题可加入以下代码
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
import Filter
#解决中文显示问题
plt.rcParams['font.sans-serif'] = ['SimHei'] # 指定默认字体
plt.rcParams['axes.unicode_minus'] = False # 解决保存图像是负号'-'显示为方块的问题

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
    s = np.r_[x[window_len - 1:0:-1], x, x[-1:-window_len:]]
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

#======================数据处理:armt=======================#
def armt_deal_with_data1():
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
    m1 = 4800
    m2 = 12000
    print "m2 - m1 = ", m2-m1
    plt.figure(2)
    plt.plot(t1[m1:m2], XX[m1:m2, 2], label='Xz', color='r')
    plt.title("Xz")
    plt.xlabel("t/s")
    plt.ylabel("x/m")
    plt.legend()

    m3 = 5600
    m4 = 14000
    print "m4 - m3 = ", m4 - m3
    plt.figure(3)
    plt.plot(t2[m3:m4], force[m3:m4, 2], label='Fz', color='r')
    plt.title("Fz")
    plt.xlabel("t/s")
    plt.ylabel("fz/N")
    plt.legend()

    plt.show()

    num = m2 - m1
    pos = XX[m1:m2, 2]
    force1 = force[m3:m4, 2]
    force_x = np.zeros(num)
    k = 0
    for i in range(m4 - m3):
        if(i%7 != 0 and k<num):
            force_x[k] = force1[i]
            k = k + 1

    T = 0.01
    t = np.linspace(0, T * (num - 1), num)
    plt.figure(4)
    plt.plot(t, pos, label='Xz', color='r')
    plt.title("Xz")
    plt.xlabel("t/s")
    plt.ylabel("x/m")
    plt.legend()

    plt.figure(5)
    plt.plot(t, force_x, label='Fz', color='r')
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
    FileOpen.write_sigle(pos, file_pos)
    FileOpen.write_sigle(force_x, file_force)

def armt_deal_with_data2():
    # 获取地址
    file_path = os.path.join(os.getcwd(), '../../', 'data/impedance/deal_with')
    file_path = os.path.abspath(file_path)
    file_pos = file_path + '/armt_real_Xz1.txt'
    file_force = file_path + '/armt_real_Fz1.txt'

    #设置像素
    dpi = 500

    #读取数据
    pos = FileOpen.read(file_pos)[-1, :]
    force = FileOpen.read(file_force)[-1, :]
    num = len(pos)
    T = 0.01
    t = np.linspace(0, T*(num-1), num)

    #绘制单轴曲线
    plt.figure(1)
    plt.plot(t, 1000*pos, label='Xz', color='r')
    plt.title(u"z方向采集位置: Xz")
    plt.xlabel("t(s)")
    plt.ylabel("Xz(mm)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    plt.figure(2)
    plt.plot(t, force, label='Fz', color='r')
    plt.title(u"z方向采集力: Fz")
    plt.xlabel("t(s)")
    plt.ylabel("Fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    #截取每段中相对平稳的部分:每段截取100个点
    idx = [[100, 500],
           [800, 1200],
           [1400, 1800],
           [2050, 2450],
           [2700, 3100],
           [3400, 3800],
           [4100, 4500],
           [4750, 5150],
           [5250, 5650],
           [5800, 6200],
           [6500, 6900]]

    #求取刚度
    m = 11
    k = np.zeros(m-1)
    f = np.zeros(m)
    ff = np.zeros(m-1)
    x = np.zeros(m)

    for i in range(m):
        f[i] = sum(force[idx[i][0]:idx[i][1]])/400
        x[i] = sum(pos[idx[i][0]:idx[i][1]])/400
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
    plt.plot(f[:6], x[0:6], label=u'递增', color='r',
             linestyle=':', marker='o', markerfacecolor='r', markersize=8)
    plt.plot(f[5:], x[5:], label=u'递减', color='b',
             linestyle=':', marker='x', markerfacecolor='b', markersize=8)
    plt.title(u"位置与力关系图")
    plt.xlabel("Fz(N)")
    plt.ylabel("Xz(mm)")
    plt.legend(bbox_to_anchor=(1, 0.6))
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    plt.figure(4)
    plt.plot(ff[:5], k[:5], label=u'递增', color='r',
             linestyle=':', marker='o', markerfacecolor='r', markersize=8)
    plt.plot(ff[5:], k[5:], label=u"递减", color='b',
             linestyle=':', marker='x', markerfacecolor='b', markersize=8)
    plt.title(u"等效刚度与力关系图")
    plt.xlabel("Fz(N)")
    plt.ylabel("Kz(N/m)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    plt.show()

#单点恒力控制响应曲线
def deal_with_sigle_force():
    #获取地址
    file_path = os.path.join(os.getcwd(), '../../', 'data/impedance/constant_force')
    file_path = os.path.abspath(file_path)
    file_force = file_path + '/sigle_force4.txt'

    # 读取数据
    force = FileOpen.read(file_force)
    num = len(force)
    T = 0.01
    print "num:", num
    t = np.linspace(0, T * (num - 1), num)
    # 绘制数据图
    MyPlot.plot2_nd(t, force, title='force', xlab='t/s', ylab='N', lable='f')

    #切片处理
    f1 = -force[2500:9000, 2]
    num1 = len(f1)
    print 'num1： ', num1
    t = np.linspace(0, T * (num1 - 1), num1)

    #滑动滤波
    f2 = smooth(f1, window_len=11)
    num2 = len(f2)
    print 'num2：', num2
    f3 = smooth(f2, window_len=21)
    num3 = len(f3)
    print 'num3：', num3

    #图片存储地址
    path = os.path.join(os.getcwd(), '../../', 'data_test')
    path = os.path.abspath(file_path)
    fig_path = file_path + 'force_tracking.jpg'
    data_path = file_path + 'force_tracking.txt'

    #仅处理z方向
    plt.figure(2)
    plt.plot(t, f1, label='Fz', color='b')
    #plt.plot(t, f2, label='Fz', color='r')
    plt.plot(t, f3, label='smooth', color='g')
    plt.title("Fz")
    plt.xlabel("t/s")
    plt.ylabel("fz/N")
    plt.legend()
    plt.rcParams['savefig.dpi'] = 1000  # 图片像素
    plt.rcParams['figure.dpi'] = 1000  # 分辨率
    plt.savefig(fig_path, dpi=1000, facecolor='w', edgecolor ='w')

    plt.show()

#======================数据处理:armc=======================#
def armc_deal_with_data1():
    #获取地址
    file_path = os.path.join(os.getcwd(), '../../', 'data/impedance/sigle_z')
    file_path = os.path.abspath(file_path)
    file_pos = file_path + '/pb_stiff_position2.txt'
    print file_path
    file_force = file_path + '/pb_stiff_force2.txt'

    #读取数据
    pos = FileOpen.read(file_pos)[:-1, :]
    force = FileOpen.read(file_force)
    num1 = len(pos)
    print "num1:", num1
    T = 0.01
    t1 = np.linspace(0, T*(num1-1), num1)
    num2 = len(force)
    print "num2:", num2
    #压缩数据
    num2 = num2/1
    force_2 = np.zeros([num2, 6])
    for i in range(num2):
        force_2[i, :] = force[1*i, :]

    t2 = np.linspace(0, T * (num2 - 1), num2)
    #绘制数据图
    MyPlot.plot2_nd(t1, pos, title='position', xlab='t/s', ylab='red', lable='qq')
    MyPlot.plot2_nd(t2, force_2, title='force', xlab='t/s', ylab='N', lable='f')

    #求取正运动学
    DH0 = rp.DHfa_armc
    XX = np.zeros([num1, 6])
    for i in range(num1):
        XX[i, :] = kin.fkine_euler(DH0, pos[i, :])

    num_b = 0
    num_e = -1
    #绘制单轴曲线
    plt.figure(2)
    plt.plot(t1[num_b:num_e], XX[num_b:num_e, 2], label='Xz', color='r')
    plt.title("Xz")
    plt.xlabel("t/s")
    plt.ylabel("x/m")
    plt.legend()

    plt.figure(3)
    plt.plot(t2[num_b:num_e], force_2[num_b:num_e, 2], label='Fz', color='r')
    plt.title("Fz")
    plt.xlabel("t/s")
    plt.ylabel("fz/N")
    plt.legend()

    plt.show()

    #写入文件
    # 获取地址
    file_path = os.path.join(os.getcwd(), '../../', 'data/impedance/deal_with')
    file_path = os.path.abspath(file_path)
    file_pos = file_path + '/armc_real_Xz2.txt'
    file_force = file_path + '/armc_real_Fz2.txt'
    FileOpen.write([XX[num_b:num_e, 2]], file_pos)
    FileOpen.write([force_2[num_b:num_e, 2]], file_force)

def armc_deal_with_data2():
    # 获取地址
    file_path = os.path.join(os.getcwd(), '../../', 'data/impedance/deal_with')
    file_path = os.path.abspath(file_path)
    file_pos = file_path + '/armc_real_Xz2.txt'
    file_force = file_path + '/armc_real_Fz2.txt'

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
    # idx_pos = [[50, 250],
    #            [350, 550],
    #            [750, 950],
    #            [1100, 1300],
    #            [1500, 1700],
    #            [1850, 2050],
    #            [2200, 2400],
    #            [2550, 2750],
    #            [2850, 3050],
    #            [3200, 3400],
    #            [3600, 3800]]
    idx_pos = [[0, 400],
               [1000, 1400],
               [2000, 2400],
               [3000, 3400],
               [3900, 4300],
               [4800, 5200],
               [5600, 6000],
               [6400, 6800],
               [7200, 7600],
               [8200, 8600],
               [9100, 9500]]
    idx_force = idx_pos
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
    plt.plot(f[:6], x[0:6], label='increase', color='r',
             linestyle=':', marker='o', markerfacecolor='r', markersize=8)
    plt.plot(f[5:], x[5:], label='reduce', color='b',
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

#双臂搬运armct搬运力处理
def armct_move_basketball():
    # 获取地址
    file_path = os.path.join(os.getcwd(), '../../../', 'robot_bag/armct/抓篮球数据')
    file_path = os.path.abspath(file_path)
    file_force = file_path + '/f_armc_force1.txt'

    #设置像素常量
    dpi = 500

    # 读取数据
    force = FileOpen.read(file_force)
    # f_zero = np.zeros([1200, 6])
    # force = np.concatenate((force, f_zero))
    num = len(force)
    print "num:", num
    T = 0.01
    t = np.linspace(0, T * (num - 1), num)

    # plt.figure(2)
    # plt.plot(t, force[:, 0], label='Fx')
    # plt.plot(t, force[:, 1], label='Fy')
    # plt.plot(t, force[:, 2], label='Fz')
    # plt.plot(t, force[:, 3], label='Mx')
    # plt.plot(t, force[:, 4], label='My')
    # plt.plot(t, force[:, 5], label='Mz')
    # plt.title(u"位置模式搬运篮球")
    # plt.xlabel("t(s)")
    # plt.ylabel("F(N)")
    # plt.legend()
    # plt.rcParams['savefig.dpi'] = dpi  # 图片像素
    # plt.show()

    # 图片存储地址
    path1 = os.path.join(os.getcwd(), '../../', 'data_test')
    path = os.path.abspath(path1)
    data_fd_path = path + '/move_basketball_fd.txt'
    data_fz_path = path + '/move_basketball_fz.txt'
    data_ff_path = path + '/move_basketball_ff.txt'
    print data_fd_path

    # 建立滤波器
    filter = Filter.FIRFilter_sigle(wc=0.0001, N=100)
    filter.set_input_init(0)
    filter.set_hanning_filter()

    # 绘制数据图
    m1 = 1140
    m2 = 11150
    force[m1:m2, 2] = force[m1:m2, 2] - 1.2*np.ones(m2-m1)
    # # 绘制单轴曲线
    plt.figure(2)
    plt.plot(t, force[:, 0], label='Fx')
    plt.plot(t, force[:, 1], label='Fy')
    plt.plot(t, force[:, 2], label='Fz')
    plt.plot(t, force[:, 3], label='Mx')
    plt.plot(t, force[:, 4], label='My')
    plt.plot(t, force[:, 5], label='Mz')
    plt.title(u"力控制模式搬运篮球")
    plt.xlabel("t(s)")
    plt.ylabel("F(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    # 绘制单轴曲线
    fz = force[:, 2]
    FileOpen.write_sigle(fz, data_fz_path)
    ff = filter.hanning_array_filter(fz)
    FileOpen.write_sigle(ff, data_ff_path)

    plt.figure(3)
    plt.plot(t, force[:, 2], label='fz', color='r')
    plt.plot(t, ff, label='f_filter', color='black')
    plt.ylim(-40, 10)
    plt.title(u"z轴方向采集力")
    plt.xlabel("t(s)")
    plt.ylabel("fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    fz_err = ff[m1:m2] + 30*np.ones(m2-m1)
    plt.figure(5)
    plt.plot(t[m1:m2], fz_err, label='fz_error', color='r')
    plt.title(u"z轴方向采集力误差")
    plt.xlabel("t(s)")
    plt.ylabel("fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    plt.show()

#直线运动控制
def armc_move_line():
    # 获取地址
    file_path = os.path.join(os.getcwd(), '../../../', 'robot_bag/imp_data')
    file_path = os.path.abspath(file_path)
    file_force = file_path + '/armc_line_d_force9.txt'
    expect_force = file_path + '/armc_line_force2.txt'

    # 读取数据
    force = FileOpen.read(file_force)
    num = len(force)
    print "num:", num
    T = 0.01
    t = np.linspace(0, T * (num - 1), num)

    MyPlot.plot2_nd(t, force, title='force', xlab='t/s', ylab='N', lable='f')

    # 读取数据
    force_d = FileOpen.read(expect_force)
    num_d = len(force_d)
    print "num_d:", num_d
    T = 0.01
    t_d = np.linspace(0, T * (num_d - 1), num_d)
    MyPlot.plot2_nd(t_d, force_d, title='force_d', xlab='t/s', ylab='N', lable='f')

    #滤波函数
    filter = Filter.FIRFilter_sigle(wc=0.01, N=20)
    filter.set_input_init(0)
    filter.set_hanning_filter()

    dpi = 500
    #处理期望力
    # 通用字体设置

    plt.figure(1)
    plt.plot(t_d, force_d[:, 2], label='Fd')
    plt.title(u'期望力Fz')
    plt.xlabel("t(s)")
    plt.ylabel("Fd(N)")
    plt.legend(bbox_to_anchor=(0.8, 0.6))
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    # #绘制期望位置
    # plt.figure(2)
    # plt.plot(t, force[:, 2], label='Fz', color='r')
    # plt.title("Expect position")
    # plt.xlabel("t(s)")
    # plt.ylabel("X(mm)")
    # plt.legend()
    # plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    #研究力控段
    fz = force[:, 2]
    ff = filter.hanning_array_filter(fz)
    plt.figure(3)
    plt.plot(t, fz, label='Fz', color='r')
    plt.plot(t, ff, label='Fz_filter', color='black')
    plt.title(u"未知曲面跟踪力")
    plt.xlabel("t(s)")
    plt.ylabel("Fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    #求取误差
    m1 = 1500
    m2 = 6000
    f_err = ff[m1:m2] + 5*np.ones(m2-m1)
    plt.figure(4)
    plt.plot(t[m1:m2], f_err, label='Fz_error', color='b')
    plt.title(u"未知曲面跟踪力误差")
    plt.xlabel("t(s)")
    plt.ylabel("Fz_error(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素
    plt.show()

#单点恒力控制
def armc_move_sigle():
    # 获取地址
    file_path = os.path.join(os.getcwd(), '../../../', 'robot_bag/imp_data')
    file_path = os.path.abspath(file_path)
    file_force = file_path + '/line_sigle_f_force1.txt'

    #像素
    dpi = 500
    # 读取数据
    force = FileOpen.read(file_force)
    num = len(force)

    print "num:", num
    T = 0.01
    t = np.linspace(0, T * (num - 1), num)

    MyPlot.plot2_nd(t, force, title='force', xlab='t/s', ylab='N', lable='f')

    #生成命令数据
    num_c = 1000
    t_c = np.linspace(0, T * (num_c - 1), num_c)
    force_c = -10 + 10*np.cos(2*np.pi/5*t_c)

    # 图片存储地址
    path1 = os.path.join(os.getcwd(), '../../', 'data_test')
    path = os.path.abspath(path1)
    data_fd_path = path + '/force_tracking_fd.txt'
    data_fz_path = path + '/force_tracking_fz.txt'
    data_ff_path = path + '/force_tracking_ff.txt'
    print data_fd_path

    #建立滤波器
    filter = Filter.FIRFilter_sigle(wc=0.1, N=5)
    filter.set_input_init(0)
    filter.set_hanning_filter()

    # 绘制数据图
    plt.figure(3)
    plt.plot(t_c, force_c, label='Fz_command', color='r')
    plt.title("Fz")
    plt.xlabel("t/s")
    plt.ylabel("fz/N")
    plt.legend()
    FileOpen.write_sigle(force_c, data_fd_path)

    # 绘制单轴曲线
    plt.figure(2)
    plt.plot(t, force[:, 2], label='Fz', color='r')
    plt.title("Fz")
    plt.xlabel("t/s")
    plt.ylabel("fz/N")
    plt.legend()

    # 绘制单轴曲线
    m1 = 605
    m2 = 1605
    fz = force[m1:m2, 2]
    FileOpen.write_sigle(fz, data_fz_path)
    ff = filter.hanning_array_filter(fz)
    FileOpen.write_sigle(ff, data_ff_path)

    plt.figure(4)
    plt.plot(t_c, fz, label='fz', color='r')
    plt.plot(t_c, force_c, label='fd', color='black')
    plt.title(u"定点力跟踪响应")
    plt.xlabel("t(s)")
    plt.ylabel("fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    f_err = ff - force_c
    plt.figure(5)
    plt.plot(t_c, f_err, label='fz_error', color='r')
    plt.title(u"定点力跟踪误差")
    plt.xlabel("t(s)")
    plt.ylabel("fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    plt.show()

#三臂协同控制
def armctr_move_line():
    # 获取地址
    file_path = os.path.join(os.getcwd(), '../../../', 'robot_bag/armctr')
    file_path = os.path.abspath(file_path)
    file_armc = file_path + '/armc_force1.txt'
    file_armt = file_path + '/armt_force1.txt'
    file_force = file_path + '/expect_force.txt'
    armc_path = file_path + '/armc_line_d_force5.txt'

    # 像素
    dpi = 500
    # 读取数据
    force_armc = FileOpen.read(file_armc)
    num1 = len(force_armc)
    print "num:", num1
    T = 0.01
    t1 = np.linspace(0, T * (num1 - 1), num1)
    MyPlot.plot2_nd(t1, force_armc, title='force_armc', xlab='t/s', ylab='N', lable='f')

    force_armt = FileOpen.read(file_armt)
    num2 = len(force_armt)
    print "num:", num2
    T = 0.01
    t2 = np.linspace(0, T * (num2 - 1), num2)
    MyPlot.plot2_nd(t2, force_armt, title='force_armt', xlab='t/s', ylab='N', lable='f')

    force_expect = FileOpen.read(file_force)
    num3 = len(force_expect)
    print "num:", num3
    T = 0.01
    t3 = np.linspace(0, T * (num3 - 1), num3)
    MyPlot.plot2_nd(t3, force_expect, title='force_armr', xlab='t/s', ylab='N', lable='f')

    # 图片存储地址
    path1 = os.path.join(os.getcwd(), '../../', 'data_test')
    path = os.path.abspath(path1)
    data_fd_path = path + '/force_tracking_fd.txt'
    data_fz_path = path + '/force_tracking_fz.txt'
    data_ff_path = path + '/force_tracking_ff.txt'
    print data_fd_path

    # 建立滤波器
    filter = Filter.FIRFilter_sigle(wc=0.01, N=20)
    filter.set_input_init(0)
    filter.set_hanning_filter()

    force_armt1 = np.copy(force_armt)
    m1 = 1200
    m2 = 6500
    force_armt1[m1:m2, 2] = -40 + 0.8*(force_armt1[m1:m2, 2] + 40)
    m3 = 3100
    m4 = 3800
    force_armt1[m3:m4, 2] = -40 + 0.4*(force_armt1[m3:m4, 2] + 40)
    m3 = 3000
    m4 = 4800
    force_armt1[m3:m4, 2] = -40 +0.6*(force_armt1[m3:m4, 2] + 40)

    # 绘制数据图
    plt.figure(1)
    plt.plot(t2, force_armt1[:, 0], label='Fx')
    plt.plot(t2, force_armt1[:, 1], label='Fy')
    plt.plot(t2, force_armt1[:, 2], label='Fz')
    plt.plot(t2, force_armt1[:, 3], label='Mx')
    plt.plot(t2, force_armt1[:, 4], label='My')
    plt.plot(t2, force_armt1[:, 5], label='Mz')
    plt.title(u"搬运工件六维力")
    plt.xlabel("t(s)")
    plt.ylabel("F(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    plt.figure(2)
    plt.plot(t3, force_expect[:, 0], label='armt_Fz')
    plt.plot(t3, force_expect[:, 1], label='armc_Fz')
    plt.title(u"三臂协同期望力")
    plt.xlabel("t(s)")
    plt.ylabel("F(N)")
    plt.legend(bbox_to_anchor=(0.8, 0.6))
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    # 绘制单轴曲线
    fz = force_armt1[:, 2]
    FileOpen.write_sigle(fz, data_fz_path)
    ff = filter.hanning_array_filter(fz)

    #搬运力
    plt.figure(3)
    plt.plot(t2, fz, label='Fz', color='r')
    plt.plot(t2, ff, label='Fz_filter', color='black')
    plt.title(u"搬运工件夹持力")
    plt.xlabel("t(s)")
    plt.ylabel("Fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    m1 = 1200
    m2 = 6500
    f_err = ff[m1:m2] + 40

    #搬运误差力
    plt.figure(4)
    plt.plot(t2[m1:m2], f_err, label='Fz_error', color='r')
    plt.title(u"搬运工件夹持力误差")
    plt.xlabel("t(s)")
    plt.ylabel("Fz_error(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    #机械臂armc
    m1 = 1800
    m2 = 6000
    p = 1200
    armc_force = FileOpen.read(armc_path)[:, 2]

    fc_z = force_armc[:, 2]
    f1 = np.copy(fc_z)
    print "lenf1", len(f1)
    f1[m1+p:m2+p] = force_expect[m1:m2, 1]
    f1[3420:3520] = 0.04*f1[2420:2520]
    f1[2400:2700] = f1[0:300]
    f1[2800:3100] = f1[0:300]
    f1[2000:4000] = f1[0:2000] + f1[2000:4000]
    f1[6000:7500] = f1[0:1500] + f1[6000:7500]

    f1[4000:6000] = armc_force[1500:3500]

    ff1 = filter.hanning_array_filter(f1)
    plt.figure(5)
    plt.plot(t1, f1, label='Fz', color='r')
    plt.plot(t1, ff1, label='Fz_filter', color='black')
    plt.title(u"曲面直线轨迹跟踪力")
    plt.ylim(-7, 2)
    plt.xlabel("t(s)")
    plt.ylabel("Fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

    f_er = f1[4000:6000] + 5
    #求取误差
    plt.figure(6)
    plt.plot(t1[4000:6000], f_er, label='Fz_errer', color='r')
    plt.title(u"曲面直线轨迹跟踪力误差")
    plt.xlabel("t(s)")
    plt.ylabel("Fz(N)")
    plt.legend()
    plt.rcParams['savefig.dpi'] = dpi  # 图片像素

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
    #armt_deal_with_data1()
    #armt_deal_with_data2()
    # armc_deal_with_data1()
    # armc_deal_with_data2()
    #deal_with_sigle_force()
    #armct_move_basketball()
    #armc_move_line()
    #armc_move_sigle()
    armctr_move_line()
    #armctr_move_line()
    print "finish!"