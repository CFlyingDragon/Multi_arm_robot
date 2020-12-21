#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于界面相关函数
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.12.21
#系统函数
import numpy as np
from math import pi

# 5次多项式插值求取求取一段时间的离散数值点,多变量
def interp5rdPoly(q0, qv0, qa0, qf, qvf, qaf, tf, dt):
    '''
        本函数用5项插值将离散数据连续化，多变量
        input:q0,qv0,qa0起点位置速度加速度
              qf,qvf,qaf终点位置速度加速度
              tf终点时间，dt时间间隔
        output:[q,qv,qa] 对应时刻位置速度加速度
    '''
    n = len(q0)
    k = np.floor(tf / dt).astype(int) + 1

    # 求取5次多项式插值系数
    a0 = np.zeros(n)
    a1 = np.zeros(n)
    a2 = np.zeros(n)
    a3 = np.zeros(n)
    a4 = np.zeros(n)
    a5 = np.zeros(n)
    for i in range(n):
        a0[i] = q0[i]
        a1[i] = qv0[i]
        a2[i] = qa0[i] / 2.0
        a3[i] = (20 * (qf[i] - q0[i]) - (8 * qvf[i] + 12 * qv0[i]) * tf + (qaf[i] - 3 * qa0[i]) * tf * tf) / (
        2 * pow(tf, 3))
        a4[i] = (-30 * (qf[i] - q0[i]) + (14 * qvf[i] + 16 * qv0[i]) * tf - (2 * qaf[i] - 3 * qa0[i]) * tf * tf) / (
        2 * pow(tf, 4))
        a5[i] = (12 * (qf[i] - q0[i]) - (60 * qvf[i] + 6 * qv0[i]) * tf + (qaf[i] - qa0[i]) * tf * tf) / (
        2 * pow(tf, 5))

    # 数据点求取
    qq = np.zeros([k, n])
    qv = np.zeros([k, n])
    qa = np.zeros([k, n])
    t_seq = np.linspace(0, tf, k)
    for i in range(k):
        t = t_seq[i]
        for j in range(n):
            qq[i, j] = a0[j] + a1[j] * t + a2[j] * pow(t, 2) + a3[j] * pow(t, 3) + a4[j] * pow(t, 4) + a5[j] * pow(t, 5)
            qv[i, j] = a1[j] + 2 * a2[j] * t + 3 * a3[j] * pow(t, 2) + 4 * a4[j] * pow(t, 3) + 5 * a5[j] * pow(t, 4)
            qa[i, j] = 2 * a2[j] + 6 * a3[j] * t + 12 * a4[j] * pow(t, 2) + 20 * a5[j] * pow(t, 3)
    return [qq, qv, qa]

#=====================从一个起点到另一个点=====================#
def pos1_to_pos2(qq1,qq2,T,tf):
	n = len(qq1)
	q0 = np.zeros(n)
	[qq, qv, qa] = interp5rdPoly(qq1, q0, q0, qq2, q0, q0, tf, T)
	return [qq, qv, qa]

# ===============================规划模块================================#
#主界面中，用于规划关节空间点到点的运动
def q_joint_space_plan(qq_b, qq_e, T=0.01):
    '''
    :param qq_b: 规划起始点
    :param qq_e: 规划终止点
    :return:
    '''
    #给定运动速度5rmp/min
    vel = 5*2*pi/60.0
    #关节角度转换到弧度单位rad
    qqb = qq_b
    qqe = qq_e
    #计算总时长

    q_max = np.max(np.abs(qqe - qqb))
    t = q_max/vel
    if(t < 10):
        t = 10
    [qq,qv,qa] = pos1_to_pos2(qqb, qqe, T, t)
    return [qq, qv, T]

#关节空间规划,给定时间
def q_joint_space_plan_time(qq_b, qq_e, T=0.01,t=10):
    '''
    :param qq_b:
    :param qq_e:
    :param T:
    :param t:
    :return:
    '''
    [qq, qv, qa] = pos1_to_pos2(qq_b, qq_e, T, t)
    return [qq, qv, qa]




