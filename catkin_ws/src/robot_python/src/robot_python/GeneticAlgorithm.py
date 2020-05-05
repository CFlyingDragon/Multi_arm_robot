#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档参考“浮点数编码的遗传算法及运用”，哈尔滨工业大学学报
#用遗传算法优化SRS型机械臂的臂型角，参数化关节角等
#程序员：陈永厅
#版权：哈尔滨工业大学
# 日期：初稿：#日期：初稿：2019.2.12 2.24 2.25
#      改进2019/12/6，增加了单变量

import numpy as np
import numpy.linalg as nla
import math
from math import pi
import time

#导入自定义函数
import BaseFunction as bf
import Kinematics as kin
import RobotParameter as rp

#=================建立初始种群Initial population,单变量=================#
def initial_pop_single(num, thetaMax, thetaMin, theta_k, sigma= 0.001):
    '''
        初始化方法采用正太分布的数据，以上一个关节角为均值，设定方差
        参数num为种群数量,thetaMax、thetaMin为关节角的取值范围
        theta_k为上一时刻的关节角,sigma为给定方差， 默认值 0.1
    '''
    theta = np.zeros(num)
    for i in range(num):
        s = np.random.normal(theta_k, sigma)  # 生成正太随机数

        # 将数据压缩到范围内，大于最大边界的取最大值，小于最小边界取最小值
        if s > thetaMax:
            theta[i] = thetaMax
        elif s < thetaMin:
            theta[i] = thetaMin
        else:
            theta[i] = s
    return theta


#=================建立初始种群Initial population=================#
def initial_pop(num, thetaMax, thetaMin, theta_k, sigma=0.5):
    '''
        初始化方法采用正太分布的数据，以上一个关节角为均值，设定方差
        参数num为种群数量,thetaMax、thetaMin为关节角的取值范围
        theta_k为上一时刻的关节角,sigma为给定方差， 默认值 0.001
    '''
    l = len(theta_k)
    Theta = np.zeros([num, l])
    for i in range(num):
        for j in range(l):
            s = np.random.normal(theta_k[j], sigma)  # 生成正太随机数
            # 将数据压缩到范围内，大于最大边界的取最大值，小于最小边界取最小值
            if s > thetaMax[j]:
                Theta[i, j] = thetaMax[j]
            elif s < thetaMin[j]:
                Theta[i, j] = thetaMin[j]
            else:
                Theta[i, j] = s
    return Theta

#=================选择操作,单变量=================#
def selection_single(theta, fit, elim_rate = 0.2):
    '''
        采用排序选择，序号代表适应度，选择前后总数不变
        将开头的m个复制两份，末尾的m个淘汰,中间的复制一份
        Theta为评判关节,Fit对应的适应度值,elim_rate淘汰率
    '''
    n = len(theta)
    m = np.int(np.floor(n * elim_rate))
    fit = list(fit)

    # 按适应值由小到大排序
    theta_sort = np.zeros(n)
    for i in range(n):
        k = fit.index(max(fit))
        theta_sort[n - 1 - i] = theta[k]
        if i == n - 1:
            fit_min = fit[k]  # 找出最佳适应度值
        fit[k] = -1  # 标记已经排序过的

    # 选择操作：去除末尾的m组，同时用最前面的m组替代
    theta = theta_sort
    for i in range(m):
        theta[n - 1 - i] = theta_sort[i]
    return [theta, fit_min]

#=================选择操作=================#
def selection(Theta, Fit, elim_rate = 0.2):
    '''
        采用排序选择，序号代表适应度，选择前后总数不变
        将开头的m个复制两份，末尾的m个淘汰,中间的复制一份
        Theta为评判关节,Fit对应的适应度值,elim_rate淘汰率
    '''
    [n, l] = np.shape(Theta)
    m = np.int(np.floor(n * elim_rate))
    Fit = list(Fit)
    # 按适应值由小到大排序
    Theta_sort = np.zeros([n, l])
    for i in range(n):
        k = Fit.index(max(Fit))
        Theta_sort[n - 1 - i, :] = Theta[k, :]
        if i == n - 1:
            fit_min = Fit[k]  # 找出最佳适应度值
        Fit[k] = -1  # 标记已经排序过的

    # 选择操作：去除末尾的m组，同时用最前面的m组替代
    Theta = Theta_sort
    for i in range(m):
        Theta[n - 1 - i, :] = Theta_sort[i, :]
    return [Theta, fit_min]

#=================交叉操作crossover,单变量=================#
def crossover_single(theta_coss, thetaMax, thetaMin, alpha_coss=0.3):
    '''
        两两配对，采用一致交叉操作
        通过交叉变化系数控制交叉的偏离大小
        如果输入为奇数组数据，最后一组不参与配对
    '''
    l = np.int(np.floor(len(theta_coss) / 2))  # 求取配对数
    N_theta = theta_coss.copy()                # 采用深拷贝初始化新据
    for j in range(l):
        i = 2 * j
        x_alpha = alpha_coss * np.random.rand()
        x_beta = alpha_coss * np.random.rand()
        s1 = (1 - x_alpha) * theta_coss[i] + x_beta * theta_coss[i + 1]
        s2 = (1 - x_beta) * theta_coss[i + 1] + x_alpha * theta_coss[i]
        ##压缩到取值范围内
        N_theta[i] = np.fmin(np.fmax(s1, thetaMin), thetaMax)
        N_theta[i+1] = np.fmin(np.fmax(s2, thetaMin), thetaMax)
    return N_theta

#=================交叉操作crossover=================#
def crossover(Theta_coss, thetaMax, thetaMin, alpha_coss=0.3):
    '''
        两两配对，采用一致交叉操作
        通过交叉变化系数控制交叉的偏离大小
        如果输入为奇数组数据，最后一组不参与配对
    '''
    m = len(thetaMax)
    l = np.int(np.floor(len(Theta_coss[:, 0]) / 2))  # 求取配对数
    N_theta = Theta_coss.copy()  # 采用深拷贝初始化新据
    for ii in range(l):
        i = 2 * ii
        x_alpha = alpha_coss * np.random.rand()
        x_beta = alpha_coss * np.random.rand()
        s1 = (1 - x_alpha) * Theta_coss[i, :] + x_beta * Theta_coss[i + 1, :]
        s2 = (1 - x_beta) * Theta_coss[i + 1, :] + x_alpha * Theta_coss[i, :]
        ##压缩到取值范围内
        for j in range(m):
            N_theta[i, j] = np.fmin(np.fmax(s1[j], thetaMin[j]), thetaMax[j])
            N_theta[i + 1, j] = np.fmin(np.fmax(s2[j], thetaMin[j]), thetaMax[j])
    return N_theta

#=================变异操作mutation,单变量=================#
def mutation_single(theta_mut, thetaMax, thetaMin):
    '''
        变异操作采用改进算法
        以输入值为中心变异
    '''
    n = len(theta_mut)
    for i in range(n):
        v = np.random.randint(2)  # 生成变异方向
        b_gama = np.random.rand()  # 生成变异偏离度
        if v == 0:
            s3 = theta_mut[i] + (thetaMax - theta_mut[i]) * b_gama
        else:
            s3 = theta_mut[i] - (theta_mut[i] - thetaMin) * b_gama
        theta_mut[i] = s3
    return theta_mut

#=================变异操作mutation=================#
def mutation(Theta_mut, thetaMax, thetaMin):
    '''
        变异操作采用改进算法
        以输入值为中心变异
    '''
    [a, b] = np.shape(Theta_mut)
    for i in range(a):
        for j in range(b):
            v = np.random.randint(2)   # 生成变异方向
            b_gama = np.random.rand()  # 生成变异偏离度
            if v == 0:
                s3 = Theta_mut[i, j] + (thetaMax[j] - Theta_mut[i, j]) * b_gama
            else:
                s3 = Theta_mut[i, j] - (Theta_mut[i, j] - thetaMin[j]) * b_gama
            Theta_mut[i, j] = s3
    return Theta_mut

#=================建立适应度函数,适应度函数具有唯一性，本函数为臂型角优化=================#
def fitness_psi(psi,psi_k,theta, theta_k, theta_max,theta_min, w1, w2):
    '''
        input:theta,当前关节值
              theta_k,上一时刻关节值
              theta_max,最大关节角
              tehta_min,最小关节角
              w1,相邻时刻平滑性优化权重
              w2,关节极限避让优化权重
              w3,臂型避障优化权重
        return:f越大，结果越坏
    '''

    n = len(theta)
    # 相邻时刻平滑性优化
    for i in range(n):
        f1 = w1[i] * bf.U_function((theta_k[i] - theta[i]),0.1,-0.1)#math.pow((theta_k[i] - theta[i])/0.1,10)

    #避关节极限优化
    for i in range(n):
        f2 = w2[i] * bf.U_function(theta[i],theta_max[i],theta_min[i])

    # 臂型避障优化

    fit = f1 + f2 + math.pow((psi - psi_k)/0.2,2)
    return fit

#=================建立适应度函数=================#
def fitness(theta, theta_k, T, T0, w, lamda, sigma):
    '''
        适应度与多个优化条件有关，即目标位置尽可能接近，关节角变化量尽可能小，上臂尽可能少运动
        theta为所需判断的关节角、theta_k为上一时刻关节角、T0为目标位置齐次矩阵,T判断齐次矩阵
        w为关节权重，原则‘上臂少动，下臂多动’,与关节角参数相同，对应相应关节，参数大减少运动
        lamda为姿态与位置权重参数，2个参数，重视目标对应参数调大
        sigma关节角优化与末端位置精度权重参数，2个参数，重视目标对应参数调大
    '''

    # 求上下时刻关节差值加权平方和
    f = 0
    for i in range(7):
        f = f + w[i] * (theta_k[i] - theta[i]) ** 2
    # 求姿态参数误差的平方和
    r = 0
    for i in range(3):
        for j in range(3):
            r = r + (T0[i, j] - T[i, j]) ** 2
    # 求位置参数误差加权平方和
    p = 0
    for k in range(3):
        p = p + (T0[k, 3] - T[k, 3]) ** 2
    # 末端姿态和位置加权
    P = lamda[0] * r + lamda[1] * p
    # 关节差值与末端误差加权求取适应度
    # 注：此处适应度值越低，个体适应度越好
    F = sigma[0] * f + sigma[1] * P
    return F

#=================建立遗传算法程序,采用FGA编码,针对臂型角参数化优化=================#
def genetic_armc_angle(Te,psi_k,theta_k,DH_0 = rp.DH0_armc,thetaMax = rp.q_max_armc,thetaMin = rp.q_min_armc):
    '''
    :param Te: 末端位姿
    :param psi_k: 上一时刻臂型角
    :param theta_k: 上一时刻关节角
    :param DH_0: 机械臂DH参数，默认armc
    :param thetaMax: 机械臂关节最大值，默认armc
    :param thetaMin: 机械臂关节最小值，默认armc
    :return: [theta_fit, psi_fit]，关节角和臂形角
    '''

    #记录计算时间
    time1 = time.clock()
    # 臂型角取值范围
    psi_min = -pi
    psi_max = pi

    # 适应度函数参数
    w1 = 10*np.array([1, 0.4, 0.4, 0.4, 0.1, 0.1, 0.1])  # 关节柔顺权重
    w2 = 0.0*np.array([0.5, 1, 0.5, 1, 0.5, 1, 0.5])      # 关节极限权重

    # 遗传参数
    pop_num = 200       # 种群大小
    generation = 300    # 遗传计算最大代数
    count = 0          # 计数
    cond_end = 5       # 连续100次适应度无提高，终止
    cross_rate = 0.5   # 交叉配对所占比例
    mut_rate = 0.1     # 变异比例
    elim_rate = 0.4    # 末尾淘汰率
    sigma = 0.001

    psi = initial_pop_single(pop_num, psi_max, psi_min, psi_k,sigma)  # 建立初始种群
    psi_fit = 0
    fitMin = 10000

    for temp in range(generation):   #计算一代
        #print "current generation: %s" % temp
        # 计算适应度值
        fit = np.zeros(pop_num)

        for i in range(pop_num):     #计算一个种群
            #print "current pop: %s" % i
            temp_theta = kin.arm_angle_all_ikine(Te, psi[i], theta_k, DH_0)
            temp_fit = 10000 * np.ones(8)  # 计算8组关节角适应度初值
            for j in range(8):       #8组解中求取其中一组
                temp_fit[j] = fitness_psi(psi[i],psi_k,temp_theta[:,j], theta_k, thetaMax,thetaMin, w1, w2)

            temp_fit = list(temp_fit)
            k_min = temp_fit.index(min(temp_fit))
            fit[i] = temp_fit[k_min]

        # 进行排序选择操作
        [psi_sort, fit_min] = selection_single(psi, fit, elim_rate)
        #print "current fitness : %s" % fit_min

        # 保留当前最佳关节值，如果连续cond_end次不改变，返回该组关节角，终止判断！
        #print "np.abs(fit_min - fitMin) = %s" % np.abs(fit_min - fitMin)
        if (np.abs(fit_min - fitMin) < math.pow(10,-6) or fit_min > fitMin):
            count = count + 1
            if count > cond_end:
                break
        if temp == 0:
            psi_fit = psi_sort[0]
            fitMin = fit_min
        else:
            if fit_min < fitMin:
                fitMin = fit_min
                psi_fit = psi_sort[0]
                count = 0  # 计数清零

        # 打乱排列顺序
        item = range(pop_num)
        items = np.copy(item)
        np.random.shuffle(items)
        psi_rand = np.zeros(pop_num)
        for i in range(pop_num):
            psi_rand[i] = psi_sort[items[i]]

        # 交叉运算
        cross_num = np.int(pop_num * cross_rate)
        psi_rand[0:cross_num] = crossover_single(psi_rand[0:cross_num], psi_max, psi_min)

        # 变异操作mutation
        mut_num = np.int(pop_num * mut_rate)
        mut_begin = cross_num
        mut_end = cross_num + mut_num
        psi_rand[mut_begin:mut_end] = mutation_single(psi_rand[mut_begin:mut_end], psi_max, psi_min)

        psi = psi_rand  # 将遗传算子操作过的种群作为下一代的父代

    #print "go here??"
    #返回最优关节角和最优臂形角
    temp_theta = kin.arm_angle_all_ikine(Te, psi_fit, theta_k, DH_0)
    temp_fit = 10000 * np.ones(8)  # 计算8组关节角适应度初值
    for j in range(8):  # 8组解中求取其中一组
        temp_fit[j] = fitness_psi(psi_fit,psi_k,temp_theta[:, j], theta_k, thetaMax, thetaMin, w1, w2)
    temp_fit = list(temp_fit)
    k_min = temp_fit.index(min(temp_fit))
    theta_fit = temp_theta[:,k_min]
    time2 = time.clock()
    print "compute one time: %s" % (time2 - time1)
    print "current psi is : %s" % psi_fit

    return [theta_fit, psi_fit]

#=================建立遗传算法程序,采用FGA编码=================#
def genetic(theta_k, T0):
    # 输入D_H已知参数
    alpha = np.array([90, 90, 90, 90, 90, 90, 0]) * (np.pi / 180)
    d = np.array([323.5, 0, 316, 0, 284.5, 0, 201]) * 0.001
    a = np.zeros(7)

    # 关节角取值范围
    thetaMax = np.array([180, 90, 180, 90, 180, 90, 180]) * (np.pi / 180)
    thetaMin = np.array([-180, -90, -180, -90, -180, -90, -180]) * (np.pi / 180)

    # 适应度函数参数
    w = np.array([40, 0.4, 0.4, 0.2, 0.1, 0.1, 0.1])  # 关节柔顺权重
    lamda = np.array([0.1, 1000])  # 姿态与位置权重参数
    sigma = np.array([0.01, 10])  # 关节角优化与末端位置精度权重参数

    # 遗传参数
    pop_num = 300  # 种群大小
    generation = 1000  # 遗传计算最大代数
    count = 0  # 计数
    cond_end = 100  # 连续100次适应度无提高，终止
    cross_rate = 0.4  # 交叉配对所占比例
    mut_rate = 0.1  # 变异比例
    elim_rate = 0.2  # 末尾淘汰率

    Theta = initial_pop(pop_num, thetaMax, thetaMin, theta_k)  # 建立初始种群

    for temp in range(generation):

        # 计算适应度值
        Fit = np.zeros(pop_num)
        for i in range(pop_num):
            T = kin.fkine(Theta[i], alpha, a, d)  # 求取位置
            Fit[i] = fitness(Theta[i], theta_k, T, T0, w, lamda, sigma)

        # 进行排序选择操作
        [Theta_sort, fit_min] = selection(Theta, Fit, elim_rate)

        # 保留当前最佳关节值，如果连续cond_end次不改变，返回该组关节角，终止判断！
        if temp == 0:
            theta_min = Theta_sort[0, :]
            FitMin = fit_min
        else:
            if fit_min < FitMin:
                FitMin = fit_min
                theta_min = Theta_sort[0, :]
                count = 0  # 计数清零
            else:
                count = count + 1
                if count > cond_end:
                    break

        # 打乱排列顺序
        item = range(pop_num)
        items = np.copy(item)
        np.random.shuffle(items)
        Theta_rand = np.zeros([pop_num, 7])
        for i in range(pop_num):
            Theta_rand[i, :] = Theta_sort[items[i], :]

        # 交叉运算
        cross_num = np.int(pop_num * cross_rate)
        Theta_rand[0:cross_num, :] = crossover(Theta_rand[0:cross_num, :],
                                               thetaMax, thetaMin)

        # 变异操作mutation
        mut_num = np.int(pop_num * mut_rate)
        mut_begin = cross_num
        mut_end = cross_num + mut_num
        Theta_rand[mut_begin:mut_end, :] = mutation(Theta_rand[mut_begin:mut_end, :], thetaMax, thetaMin)

        Theta = Theta_rand  # 将遗传算子操作过的种群作为下一代的父代

        if temp % 100 == 0:
            print '循环中:', str(temp)
            print (FitMin)
    return theta_min
