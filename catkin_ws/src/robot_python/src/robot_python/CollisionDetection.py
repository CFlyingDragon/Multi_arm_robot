#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于写碰撞检测相关算法
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2019.5.23
import numpy as np
import numpy.linalg as nla
import math
from math import pi

import BaseFunction as bf
import RobotParameter as rp
import Robots

#======机械臂之间碰撞检测,简化为胶囊体========#
#点与线段关系
def segment_point_distance(l_ab,c):
    '''
    :param l_ab:
    :param c:
    :return:
    '''
    # ab线段的点
    a = l_ab[0, :]
    b = l_ab[1, :]

    #向量
    lab = b - a
    lac = c - a

    # 求取垂点
    td = 0.0

    #距离
    d = 0.0

    #判断点c是否在ab所在直线
    cross = nla.norm(np.cross(lab, lac))
    if (cross < np.power(10,-6)):
        if(abs(lab[0]) > np.power(10, -3)):
            td = lac[0]/lab[0]
        elif(abs(lab[1]) > np.power(10, -3)):
            td = lac[1]/lab[1]
        else:
            td = lac[2]/lab[2]
        if (td < 0):
            d = nla.norm(c - a)
        elif(td > 1):
            d = nla.norm(c - b)
        else:
            d = 0.0
    else:
        td = np.dot(lab, lac)/(nla.norm(lab)**2)

        if (td < 0):
            d = nla.norm(c - a)
        elif (td > 1):
            d = nla.norm(c - b)
        else:
            xd = a + lab*td
            d = nla.norm(c - xd)
    return d

#两个线段之间的关系
def segment_distance(l_ab, l_cd):
    '''
    :param lab: R2*3
    :param lcd: R2*3
    :return:
    '''
    #ab线段的点
    a = l_ab[0, :]
    b = l_ab[1, :]

    #cd线段的点
    c = l_cd[0, :]
    d = l_cd[1, :]

    #求取线段所在向量
    lab = b - a
    lcd = d - c
    lac = c - a
    lad = d - a

    #求距离选取点
    p = np.zeros([2, 3])

    #定义线段间距离
    dd = 0.0

    #判断是直线是否共面
    det = np.dot(lab, np.cross(lac, lcd)) #三个向量混合积
    if (det < math.pow(10, -6)):#共面
        #判断是否平行
        if (nla.norm(np.cross(lab, lcd)) < math.pow(10, -6)):#平行情况
            #判断是否共线
            if (nla.norm(np.cross(lab, lac)) < math.pow(10, -6)):#共线
                #求C,D的在线段AB上的参数
                tc = 0.0
                td = 0.0
                if (abs(lab[0]) > math.pow(10, -6)):
                    tc = lac[0] / lab[0]
                    td = lad[0] / lab[0]
                elif (abs(lab[1]) > math.pow(10, -6)):
                    tc = lac[1] / lab[1]
                    td = lad[1] / lab[1]
                else:
                    tc = lac[2] / lab[2]
                    td = lad[2] / lab[2]
                #求取最短距离
                if (tc < 0 and td < 0):
                    if (tc < td):
                        dd = nla.norm(d - a)
                    else:
                        dd = nla.norm(c - a)
                elif (tc > 1 and td > 1):
                    if (tc < td):
                        dd = nla.norm(c - b)
                    else:
                        dd = nla.norm(d - b)
                else:
                    dd = 0.0

            else:#平行不共线
                #求取CD在AB上的垂点
                tc = np.dot(lab, lac) / (nla.norm(lab) ** 2)
                td = np.dot(lab, lad) / (nla.norm(lab) ** 2)
                # 求取最短距离
                if (tc < 0 and td < 0):
                    if (tc < td):
                        dd = nla.norm(d - a)
                    else:
                        dd = nla.norm(c - a)
                elif (tc > 1 and td > 1):
                    if (tc < td):
                        dd = nla.norm(c - b)
                    else:
                        dd = nla.norm(d - b)
                else:
                    dd = nla.norm(lac - lab*tc)

        else: #相交情况
            #不考虑已经相交的情况
            d_list = np.zeros(4)
            d_list[0] = segment_point_distance(l_ab, c)
            d_list[1] = segment_point_distance(l_ab, d)
            d_list[2] = segment_point_distance(l_cd, a)
            d_list[3] = segment_point_distance(l_cd, b)
            dd = min(d_list)

    else:#异面线段
        #计算垂点参数
        A = np.array([[- np.dot(lab, lab), np.dot(lab, lcd)],
                      [- np.dot(lcd, lab), np.dot(lcd, lcd)]])
        l = np.array([np.dot(lab, lac), np.dot(lcd, lac)])
        t = np.dot(nla.inv(A), l)

        #分情况赋值
        if (t[0] < 0 or t[0]==0):
            p[0, :] = a
        elif (t[0] > 1 or t[0]==1):
            p[0, :] = b
        else:
            p[0, :] = a + lab*t[0]

        if (t[1] < 0 or t[1]==0):
            p[1, :] = c
        elif (t[1] > 1 or t[1]==1):
            p[1, :] = d
        else:
            p[1, :] = a + lcd*t[1]

        #计算距离
        dd = nla.norm(p[1, :] - p[0, :])
    return dd

#三个机械臂之间碰撞检测
def collision_detection_capsule(DH_armc,DH_ur,qq_armc,qq_ur1,qq_ur2,r_armc,r_ur):
    '''
    :param DH_armc:
    :param DH_ur:
    :param qq_armc:
    :param qq_ur1:
    :param qq_ur2:
    :param r_armc:
    :param r_ur:
    :return:
    '''
    #***对机械臂覆盖胶囊体***#
    ##机械臂1,自制机械臂armc
    #正运动学
    n1 = len(DH_armc[:, 0])
    p1 = np.zeros([n1, 3])
    # 转换到世界坐标系
    A1 = Robots.base_to_world1(rp.X1)
    for i in range(n1):
        Ti = bf.trans(DH_armc[i, 0] + qq_armc[i], DH_armc[i, 1],
                      DH_armc[i, 2], DH_armc[i, 3])
        A1 = np.dot(A1, Ti)
        p1[i, :] = A1[0:3, 3]
    #线段,仅考虑可能碰撞的三段
    l_ab = np.array([[p1[0, :], p1[2, :]],
                     [p1[2, :], p1[4, :]],
                     [p1[4, :], p1[6, :]]])

    ##机械臂2、3,ur5机械臂,考虑三段
    # 正运动学
    n2 = len(DH_ur[:, 0])
    U_ur1 = np.zeros([4, 4, n2])
    U_ur2 = np.zeros([4, 4, n2])
    #转换到世界坐标系
    A_ur1 = Robots.base_to_world1(rp.X2)
    A_ur2 = Robots.base_to_world1(rp.X3)
    for i in range(n2):
        T_ur1 = bf.trans(DH_ur[i, 0] + qq_ur1[i], DH_ur[i, 1],
                         DH_ur[i, 2], DH_ur[i, 3])
        A_ur1 = np.dot(A_ur1, T_ur1)
        U_ur1[:, :, i] = A_ur1

        T_ur2 = bf.trans(DH_ur[i, 0] + qq_ur2[i], DH_ur[i, 1],
                         DH_ur[i, 2], DH_ur[i, 3])
        A_ur2 = np.dot(A_ur2, T_ur2)
        U_ur2[:, :, i] = A_ur2

    #线段,仅考虑可能碰撞的三段
    ur1_p1 = np.dot(U_ur1[:, :, 0], np.array([0, 0, DH_ur[3, 3], 1]))[0:3]
    ur1_p2 = np.dot(U_ur1[:, :, 0], np.array([DH_ur[2, 1], 0, DH_ur[3, 3], 1]))[0:3]
    ur1_p3 = U_ur1[0:3, 3, 1]
    ur1_p4 = U_ur1[0:3, 3, 2]
    ur1_p5 = U_ur1[0:3, 3, 3]
    ur1_p6 = U_ur1[0:3, 3, 4]
    l_cd = np.array([[ur1_p1, ur1_p2], [ur1_p3, ur1_p4], [ur1_p5, ur1_p6]])

    ur2_p1 = np.dot(U_ur2[:, :, 0], np.array([0, 0, DH_ur[3, 3], 1]))[0:3]
    ur2_p2 = np.dot(U_ur2[:, :, 0], np.array([DH_ur[2, 1], 0, DH_ur[3, 3], 1]))[0:3]
    ur2_p3 = U_ur2[0:3, 3, 1]
    ur2_p4 = U_ur2[0:3, 3, 2]
    ur2_p5 = U_ur2[0:3, 3, 3]
    ur2_p6 = U_ur2[0:3, 3, 4]
    l_ef = np.array([[ur2_p1, ur2_p2], [ur2_p3, ur2_p4], [ur2_p5, ur2_p6]])

    #计算线段之间的关系,并计算碰撞
    collision_flag = False
    for i in range(3):
        for j in range(3):
            #armc与UR1
            d_armc_ur1 = segment_distance(l_ab[i, :, :], l_cd[j, :, :])
            collision_flag1 = d_armc_ur1 - (r_armc[i] + r_ur[j])
            if (collision_flag1 <= 0.0):
                print "机械臂armc与UR5_1碰撞"
                collision_flag = True
                return collision_flag

            #armc与UR2
            d_armc_ur2 = segment_distance(l_ab[i, :, :], l_ef[j, :, :])
            collision_flag2 = d_armc_ur2 - (r_armc[i] + r_ur[j])
            if (collision_flag2 <= 0.0):
                print "机械臂armc与UR5_2碰撞"
                collision_flag = True
                return collision_flag

            # UR1与UR2
            d_ur1_ur2 = segment_distance(l_cd[i, :, :], l_ef[j, :, :])
            collision_flag3 = d_ur1_ur2 - (r_ur[i] + r_ur[j])
            if (collision_flag3 <= 0.0):
                print "机械臂UR5_1与UR5_2碰撞"
                collision_flag = True
                return collision_flag

    return collision_flag

#碰撞检测算法测试
def collision_test():
    #机械臂DH参数
    DH_armc = rp.DH0_armc
    DH_ur = rp.DH0_ur5

    #机械臂状态
    qq_armc = np.array([40, 70, 60, 50, 40, 80, 100]) * pi / 180
    qq_ur1 = np.array([40, 70, 60, 50, 40, 80]) * pi / 180
    qq_ur2 = np.array([-40, 70, -60, 50, 40, -80]) * pi / 180

    #仅考虑后三段,臂杆半径
    r_armc = np.array([60, 60, 60])*0.001
    r_ur = np.array([60, 60, 60]) * 0.001

    collision_flag = collision_detection_capsule(DH_armc,
                        DH_ur, qq_armc, qq_ur1, qq_ur2,
                        r_armc, r_ur)
    if (collision_flag == True):
        print "存在碰撞"


def main():
    ##基于胶囊体碰撞检测
    collision_test()

    print 'finish'


if __name__ == '__main__':
    main()
