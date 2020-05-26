#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于机器学习算法开发
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020年5月21号

import numpy as np
import numpy.linalg as nla

import math

# =========================RBF高斯径向基神经网络==========================#
#***********高斯函数计算隐藏层***************#
#高斯径向基函数计算隐藏层,单输入
def gauss_fuction_oput(xp,xc,sigma):
    '''
    :param xp: 标量
    :param xc: 一维,长度h
    :param sigma: 标量
    :return: 一维,长度h
    '''
    # 求取网络形状
    h = len(xc)
    #计算样本点到中心点距离
    dist = np.zeros(h)
    for i in range(h):
        dist[i] = abs(xp - xc[i])
    #计算高斯函数值
    phi = np.exp(-dist/(2*sigma**2))
    return phi

#高斯径向基函数计算隐藏层,多输入
def gauss_fuction_nput(xp,xc,sigma):
    '''
    :param xp: 标量
    :param xc: 矩阵
    :param sigam: 标量
    :return: 一维
    '''
    # 求取网络形状
    xc = np.array(xc)
    [h, m] = xc.shape
    #计算样本点到中心点距离
    dist = np.zeros(h)
    for i in range(h):
        dist[i] = nla.norm(xp - xc[i, :])
    #计算高斯函数值
    phi = np.exp(-dist/(2*sigma**2))
    return phi

#************正向计算*************#
#正向计算,单输入,多输出
def rbf_oput_nout(xp,xc,sigma,w):
    '''
    :param xp: 输入值,标量
    :param xc: 隐藏层中心值,一维,长度h
    :param sigma: 方差,标量
    :param w: 权重,矩阵n*h
    :return: 输出y,一维向量,长度n
    '''
    #求取隐藏层
    phi = gauss_fuction_oput(xp, xc, sigma)

    #求取输出层
    y = np.dot(w, phi)
    return y

#正向计算,多输入,多输出
def rbf_nput_nout(xp,xc,sigma,w):
    '''
    :param xp: 输入值,一维向量,m
    :param xc: 隐藏层中心值,矩阵好h*m
    :param sigma: 方差,标量
    :param w: 权重,矩阵n*h
    :return: 输出y,一维向量,长度n
    '''
    #求取隐藏层
    phi = gauss_fuction_nput(xp, xc, sigma)

    #求取输出层
    y = np.dot(w, phi)
    return y

#反向计算参数
#计算中心值及方差
def rbf_centrol():
    pass

#计算隐藏层到输出层权重
def rbf_weight_nput_nout(x,xc,sigma,y):
    '''
    :param x: 输入样本,p*m
    :param xc: rbf中心值,h*m
    :param sigma: 方差,标量
    :param y: 输出样本,p*n
    :return w:权重,n*h
    '''
    #求取样本长度
    p = len(x[:, 0])
    #隐藏层节点个数
    h = len(xc[:, 0])

    #求取隐藏层,列向扩展
    phi = np.zeros([p, h])
    for i in range(p):
        phi[i, :] = gauss_fuction_nput(x[i, :], xc, sigma)

    #采用广义逆求取权重
    w = np.dot(y.T, nla.pinv(phi.T))
    return w

# 计算隐藏层到输出层权重
def rbf_weight_oput_nout(x, xc, sigma, y):
    '''
    :param x: 输入样本,一维,长度p
    :param xc: rbf中心值,长度h
    :param sigma: 方差,标量
    :param y: 输出样本,p*n
    :return w:权重,n*h
    '''
    #求取样本长度
    p = len(x)
    #隐藏层节点个数
    h = len(xc)

    #求取隐藏层
    phi = np.zeros([p, h])
    for i in range(p):
        phi[i, :] = gauss_fuction_oput(x[i], xc, sigma)

    #采用广义逆求取权重
    w = np.dot(y.T, nla.pinv(phi.T))
    return w

#==========================k_means聚类算法==========================#
#多维数据
def k_means_nput(x,k):
    '''
    :param x: 样本,p*m
    :param k:
    :return:
    '''
    #求取数据个数
    x = np.array(x)
    [p, m] = x.shape

    #采用随机选取聚类中心
    center_index = np.random.randint(0, p, k)
    center = np.zeros([k, m])
    for i in range(k):
        center[i, :] = x[center_index[i], :]

    #迭代求取新的聚类中心
    iter_ = 100
    dis = np.zeros([p, k])
    class_flag = np.zeros(p)
    while iter_ > 0:
        # 求各个点到k个聚类中心距离
        for i in range(p):
            for j in range(k):
                dis[i, j] = nla.norm(x[i, :] - center[j, :])

            #归类,为每个数据贴上类标签
            class_flag[i] = np.argmin(dis[i, :])  # 返回最小值所在标签

        #求新的聚类中心
        center_sum = np.zeros(k)
        center_count = np.zeros(k)
        for i in range(p):
            #每个类求和
            center_sum[class_flag[i]] = center_sum[class_flag[i]] + x[i]
            #统计每个类的个数
            center_count[class_flag[i]] = center_count[class_flag[i]] + 1
        center_new = center_sum/center_count

        #判定聚类中心是否发生变换
        if all(center == center_new):
            #如果没发生变换则退出循环，表示已得到最终的聚类中心
            break

        #更新中心值
        center = center_new

    return center

