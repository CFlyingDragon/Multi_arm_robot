#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于建立多机器人相关问题
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.4.16
import numpy as np
import BaseFunction as bf

#=================求取基座标系与世界坐标系之间的齐次变换矩阵=================#
def base_to_world(X):
    '''
    :param X: 机械臂的基座六维坐标系，X为n*6
    :return:
    '''
    #建立初始值
    m = len(X[:,0]) #机械臂个数
    T = np.zeros([m,4,4])
    for i in range(m):
        #转化为齐次矩阵,RPY与zyx欧拉角等价
        T[i,0:3,0:3] = bf.euler_zyx2rot(X[i,3:6])
        T[i,0:3,3] = X[i,0:3]
        T[i,3,3] = 1
    return T

#=================工件坐标系到世界坐标系的转换=================#
def workpiece_to_world(Xw):
    '''
    :param X1: 工具坐标系转换到世界坐标系
    :return:
    '''
    # 建立初始值
    T1 = np.eye(4)

    # 转化为齐次矩阵,RPY与zyx欧拉角等价
    T1[0:3, 0:3] = bf.euler_zyx2rot(Xw[3:6])
    T1[0:3, 3] = Xw[0:3]
    return T1

#=================机械臂末端在世界坐标系中的位置=================#
def tool_to_word(C,Xw):
    '''
    :param C: 机械臂与工件接触点坐标在工件坐标系下六维表示RPY，C为n×6
    :param Xw:工件坐标系在世界坐标系下的六维表示
    :return:
    '''
    # 建立初始值
    m = len(C[:,0])
    T = np.zeros([m,4,4])
    Tw = np.eye(4)
    # 转化为齐次矩阵,RPY与zyx欧拉角等价
    # 工件
    Tw[0:3, 0:3] = bf.euler_zyx2rot(Xw[3:6])
    Tw[0:3, 3] = Xw[0:3]
    # 机械臂
    for i in range(m):
        #工具坐标系下表示
        T[i,0:3,0:3] = bf.euler_zyx2rot(C[i,3:6])
        T[i,0:3,3] = C[i,3:6]
        #转换到世界坐标系
        T[i,:,:] = np.dot(Tw,T[i,:,:])
    return T

#=================工具坐标系在基座标系中的位置=================#
def tool_to_base(X,C,Xw):
    '''
    :param X: 基座标系在世界坐标系中的六维表示
    :param C: 机械臂工具坐标系在工件坐标系下的六维表示
    :param Xw: 工件坐标系在世界坐标系下的六维表示
    :return:
    '''
    # 建立初始值
    m = len(C[:,0]) #机器人个数
    Xe = np.zeros([m,6])

    #带入函数转换为齐次矩阵
    Tw_t = tool_to_word(C,Xw)
    Tw_b = base_to_world(X)
    for i in range(m):
        Te = np.dot(np.inv(Tw_b[i,:,:]),Tw_t[i,:,:])
        Xe[i,:] = bf.T_to_Xzyx(Te)
    return Xe
