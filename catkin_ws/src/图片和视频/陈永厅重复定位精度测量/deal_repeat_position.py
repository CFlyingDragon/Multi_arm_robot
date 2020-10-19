#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档处理重复定位精度数据
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：2020年10月10日
import numpy as np
import matplotlib.pyplot as plt

# 读取数据
def read():
    ''' read 函数用于读取数据,数据由单空格隔开
        输入参数：path 读取文件的路径
        返回参数：数组
    '''
    path = "20201010Y2.txt"
    read_data = []
    with open(path, 'r') as file_to_read:
        while True:
            line = file_to_read.readline()  # 整行读取数据
            if not line:
                break

            line = line.split(",")  # 用空格拆分数据

            #line = line[:-1]  # 切片去除末尾转行符
            read_data.append(line)
    data1 = np.array(read_data[5:])[:, 1:4]
    #print "data1:", data1
    data2 = np.zeros(data1.shape)
    k = len(data2)
    n = 3
    print k
    for i in range(k):
        for j in range(n):
            if j==2:
                data1[i, j] = data1[i, j][:-2]
            data2[i, j] = data1[i, j]
    #print "data2:", data2

    #选取有效的30组数据
    m = 30
    k = 0
    kk = 5
    b = 0
    c = 0
    d = [7, 11, 22, 23, 26, 30, 33]
    data3 = np.zeros([5*m, 3])
    for i in range(m):
        while(b in d):
            print "b", b
            b = b + 1
            c = c + kk
        data3[k:k+kk, :] = data2[c:c+kk, :]
        k = k + kk
        c = c + kk
        b = b + 1
    print "data3", data3.shape

    #解析成能用的数据点
    P = np.zeros([m, 3, 5])
    k = 0
    for i in range(m):
        P[i, :, 1] = data3[k, :]
        P[i, :, 2] = data3[k+1, :]
        P[i, :, 3] = data3[k+2, :]
        P[i, :, 4] = data3[k+3, :]
        P[i, :, 0] = data3[k+4, :]
        k = k + 5
    return P

def deal_position():
    #获得数据
    P = read()
    m = len(P)
    n = 5
    # # 绘画函数,仅考虑第一维
    # plt.figure(1)
    # plt.plot(P[:, 0, 0], label='x', color='r')
    # plt.plot(P[:, 1, 0], label='y', color='g')
    # plt.plot(P[:, 2, 0], label='y', color='b')
    # plt.title("Position")
    # plt.xlabel("num")
    # plt.ylabel("x/mm")
    # plt.legend()
    # plt.show()

    #求取平均值
    M = np.zeros([n, 3])
    for i in range(n):
        for j in range(3):
            M[i, j] = np.mean(P[:, j, i])
    print "M:\n", M

    #计算偏差值
    D = np.zeros([m, n])
    for i in range(m):
        for j in range(n):
            D[i, j] = np.linalg.norm(P[i, :, j]-M[j, :])


    #print "获得偏差值：\n", D
    # 计算偏差值平均值\标准差
    m_D = np.zeros(n)
    s_D = np.zeros(n)
    for i in range(n):
        m_D[i] = np.mean(D[:, i])
        s_D[i] = np.std(D[:, i])
    print "m_D:\n", m_D
    print "s_D:\n", s_D

    #重复定位精度
    r = m_D[i] + 3.0*s_D

    print "重复定位精度：\n", r





if __name__=="__main__":
    deal_position()