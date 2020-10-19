#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于建立多机器人相关问题
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.4.16
import numpy as np
import  numpy.linalg as nla
import BaseFunction as bf
import Kinematics as kin
import RobotParameter as rp

#=================求取基座标系与世界坐标系之间的齐次变换矩阵=================#
#单臂
def base_to_world1(X):
    '''
    :param X: 机械臂的基座六维坐标系，X为6
    :return:
    '''
    #建立初始值

    T = np.zeros([4,4])
    #转化为齐次矩阵,RPY与zyx欧拉角等价
    T[0:3,0:3] = bf.euler_zyx2rot(X[3:6])
    T[0:3,3] = X[0:3]
    T[3,3] = 1
    return T

#多臂
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

#===============两个UR机械臂搬运物体类===================#
class RobotsMoveObject(object):
    def __init__(self):
        pass
    def get_robot1_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_1 = DH_0
        self.qq_min1 = qq_min
        self.qq_max1 = qq_max
        self.n1 = len(qq_min)
        self.kin1 = kin.GeneralKinematic(self.DH_0_1)

    def get_robot2_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_2 = DH_0
        self.qq_min2 = qq_min
        self.qq_max2 = qq_max
        self.n2 = len(qq_min)
        self.kin2 = kin.GeneralKinematic(self.DH_0_2)

    #机器人基座相对世界坐标系
    def get_robots_base_to_world(self, Tw_b1, Tw_b2):
        self.Tw_b1 = np.copy(Tw_b1)
        self.Tw_b2 = np.copy(Tw_b2)

    def get_robots_base_to_world_zyx(self, Xw_b1, Xw_b2):
        self.Tw_b1 = np.eye(4)
        self.Tw_b2 = np.eye(4)

        #转换为齐次矩阵
        self.Tw_b1[0:3, 3] = Xw_b1[0:3]
        self.Tw_b1[0:3, 0:3] = bf.euler_zyx2rot(Xw_b1[3:6])
        self.Tw_b2[0:3, 3] = Xw_b2[0:3]
        self.Tw_b2[0:3, 0:3] = bf.euler_zyx2rot(Xw_b2[3:6])

    #工具坐标相对物体坐标
    def get_robots_tool_to_object(self, To_t1, To_t2):
        self.To_t1 = To_t1
        self.To_t2 = To_t2

    def get_robots_tool_to_object_zyx(self,Xo_t1, Xo_t2):
        self.To_t1 =np.eye(4)
        self.To_t2 = np.eye(4)

        # 转换为齐次矩阵
        self.To_t1[0:3, 3] = Xo_t1[0:3]
        self.To_t1[0:3, 0:3] = bf.euler_zyx2rot(Xo_t1[3:6])
        self.To_t2[0:3, 3] = Xo_t2[0:3]
        self.To_t2[0:3, 0:3] = bf.euler_zyx2rot(Xo_t2[3:6])

    # 物体在世界坐标系中的位姿
    def get_object_plan_list(self, To_array):
        '''
        :param Tb_array: [num,4,4]数组
        :return:
        '''
        #获取规划点个数
        self.num = len(To_array)

        self.To_array = To_array

    def get_object_plan_list_zyx(self, Xo_array):
        '''
        :param Xb_array: [num,6]数组
        :return:
        '''
        #获取规划点个数
        self.num = len(Xo_array)

        #转换为齐次矩阵
        self.To_array = np.zeros([self.num, 4, 4])
        for i in range(self.num):
            self.To_array[i, 0:3, 0:3] = bf.euler_zyx2rot(Xo_array[i, 3:6])
            self.To_array[i, 0:3, 3] = Xo_array[i, 0:3]
            self.To_array[i, 3, 3] = 1.0

    #获得准备段大小
    def get_ready_distance(self, l, ready_num):
        self.l = l
        self.r_num = ready_num

    def tool_to_robots_base(self):
        # #---创建准备段----#
        self.r_Tb_t1 = np.zeros([self.r_num, 4, 4])
        self.r_Tb_t2 = np.zeros([self.r_num, 4, 4])

        #移动工具坐标系
        t = np.linspace(0, 1, self.r_num)
        l_array = self.l*np.cos(np.pi/2*t)

        for i in range(self.r_num):
            r_To_t1  = np.copy(self.To_t1)
            r_To_t1[0, 3] = self.To_t1[0, 3] - l_array[i]

            r_To_t2 = np.copy(self.To_t2)
            r_To_t2[0, 3] = self.To_t2[0, 3] + l_array[i]

            # 坐标变换求取工具坐标系在基座下的表示
            self.r_Tb_t1[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b1),
                                                self.To_array[0, :, :]),
                                           r_To_t1)
            self.r_Tb_t2[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b2),
                                                self.To_array[0, :, :]),
                                           r_To_t2)

        #----抱物移动端-----#
        #创建规划变量
        self.Tb_t1 = np.zeros([self.num, 4, 4])
        self.Tb_t2 = np.zeros([self.num, 4, 4])

        #坐标变换求取工具坐标系在基座下的表示
        for i in range(self.num):
            self.Tb_t1[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b1),
                                                self.To_array[i, :, :]),
                                         self.To_t1)

            self.Tb_t2[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b2),
                                                self.To_array[i, :, :]),
                                         self.To_t2)

    def put_tool_robots_bass(self, Xw_o):
        #转换为齐次矩阵
        Tw_o = np.eye(4)
        Tw_o[0:3, 0:3] = bf.euler_zyx2rot(Xw_o[3:6])
        Tw_o[0:3, 3] = Xw_o[0:3]

        #求取基座系下工具位置
        Tb_t1 = np.dot(np.dot(nla.inv(self.Tw_b1), Tw_o), self.To_t1)
        Tb_t2 = np.dot(np.dot(nla.inv(self.Tw_b2), Tw_o), self.To_t2)
        return [Tb_t1, Tb_t2]

    def put_robots_joint_position(self, q1_guess, q2_guess):
        #求取工具坐标基座标系的表示
        self.tool_to_robots_base()

        #建立关节角变量
        num = self.num +self.r_num

        qq1 = np.zeros([num, self.n1])
        qq2 = np.zeros([num, self.n2])
        qq1_guess = np.copy(q1_guess)
        qq2_guess = np.copy(q2_guess)

        Tb_t1 = np.zeros([num, 4, 4])
        Tb_t2 = np.zeros([num, 4, 4])

        Tb_t1[0:self.r_num, :, :] = np.copy(self.r_Tb_t1)
        Tb_t1[self.r_num:, :, :] = np.copy(self.Tb_t1)

        Tb_t2[0:self.r_num, :, :] = np.copy(self.r_Tb_t2)
        Tb_t2[self.r_num:, :, :] = np.copy(self.Tb_t2)


        #求取逆运动学
        for i in range(num):
            qq1[i, :] = self.kin1.ur_ikine(Tb_t1[i, :, :], qq1_guess)
            qq2[i, :] = self.kin2.ur_ikine(Tb_t2[i, :, :], qq2_guess)
            qq1_guess = qq1[i, :]
            qq2_guess = qq2[i, :]
        return [qq1, qq2]

    def put_robots_tool_position(self):
        return [self.Tb_t1, self.Tb_t2]

    def put_robots_tool_position_zyx(self):
        Xb_t1 = np.zeros(self.num, 6)
        Xb_t2 = np.zeros(self.num, 6)

        for i in range(self.num):
            Xb_t1[i, 0:3] = self.Tb_t1[i, 0:3, 3]
            Xb_t1[i, 3:6] = bf.rot2euler_zyx(self.Tb_t1[i, 0:3, 0:3])

            Xb_t2[i, 0:3] = self.Tb_t2[i, 0:3, 3]
            Xb_t2[i, 3:6] = bf.rot2euler_zyx(self.Tb_t2[i, 0:3, 0:3])

        return [Xb_t1, Xb_t2]

# ===============两个armct机械臂搬运物体类===================#
class ArmctMoveObject(object):
    def __init__(self):
        pass

    def get_robot1_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_1 = DH_0
        self.qq_min1 = qq_min
        self.qq_max1 = qq_max
        self.n1 = len(qq_min)
        self.kin1 = kin.GeneralKinematic(self.DH_0_1, self.qq_min1, self.qq_max1)

    def get_robot2_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_2 = DH_0
        self.qq_min2 = qq_min
        self.qq_max2 = qq_max
        self.n2 = len(qq_min)
        self.kin2 = kin.GeneralKinematic(self.DH_0_2, self.qq_min2, self.qq_max2)

    # 机器人基座相对世界坐标系
    def get_robots_base_to_world(self, Tw_b1, Tw_b2):
        self.Tw_b1 = np.copy(Tw_b1)
        self.Tw_b2 = np.copy(Tw_b2)

    def get_robots_base_to_world_zyx(self, Xw_b1, Xw_b2):
        self.Tw_b1 = np.eye(4)
        self.Tw_b2 = np.eye(4)

        # 转换为齐次矩阵
        self.Tw_b1[0:3, 3] = Xw_b1[0:3]
        self.Tw_b1[0:3, 0:3] = bf.euler_zyx2rot(Xw_b1[3:6])
        self.Tw_b2[0:3, 3] = Xw_b2[0:3]
        self.Tw_b2[0:3, 0:3] = bf.euler_zyx2rot(Xw_b2[3:6])

    # 工具坐标相对物体坐标
    def get_robots_tool_to_object(self, To_t1, To_t2):
        self.To_t1 = To_t1
        self.To_t2 = To_t2

    def get_robots_tool_to_object_zyx(self, Xo_t1, Xo_t2):
        self.To_t1 = np.eye(4)
        self.To_t2 = np.eye(4)

        # 转换为齐次矩阵
        self.To_t1[0:3, 3] = Xo_t1[0:3]
        self.To_t1[0:3, 0:3] = bf.euler_zyx2rot(Xo_t1[3:6])
        self.To_t2[0:3, 3] = Xo_t2[0:3]
        self.To_t2[0:3, 0:3] = bf.euler_zyx2rot(Xo_t2[3:6])

    # 物体在世界坐标系中的位姿
    def get_object_plan_list(self, To_array):
        '''
        :param Tb_array: [num,4,4]数组
        :return:
        '''
        # 获取规划点个数
        self.num = len(To_array)

        self.To_array = To_array

    def get_object_plan_list_zyx(self, Xo_array):
        '''
        :param Xb_array: [num,6]数组
        :return:
        '''
        # 获取规划点个数
        self.num = len(Xo_array)

        # 转换为齐次矩阵
        self.To_array = np.zeros([self.num, 4, 4])
        for i in range(self.num):
            self.To_array[i, 0:3, 0:3] = bf.euler_zyx2rot(Xo_array[i, 3:6])
            self.To_array[i, 0:3, 3] = Xo_array[i, 0:3]
            self.To_array[i, 3, 3] = 1.0

    # 获得准备段大小
    def get_ready_distance(self, l, ready_num):
        self.l = l
        self.r_num = ready_num

    #获得停止段点数
    def get_move_stop_num(self, ms):
        self.ms = ms

    def tool_to_robots_base(self):
        # #---创建准备段----#
        self.r_Tb_t1 = np.zeros([self.r_num, 4, 4])
        self.r_Tb_t2 = np.zeros([self.r_num, 4, 4])

        # 移动工具坐标系
        t = np.linspace(0, 1, self.r_num)
        l_array = self.l * np.cos(np.pi / 2 * t)

        for i in range(self.r_num):
            r_To_t1 = np.copy(self.To_t1)
            r_To_t1[0, 3] = self.To_t1[0, 3] + l_array[i]

            r_To_t2 = np.copy(self.To_t2)
            r_To_t2[0, 3] = self.To_t2[0, 3] - l_array[i]

            # 坐标变换求取工具坐标系在基座下的表示
            self.r_Tb_t1[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b1),
                                                  self.To_array[0, :, :]),
                                           r_To_t1)
            self.r_Tb_t2[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b2),
                                                  self.To_array[0, :, :]),
                                           r_To_t2)

        # ----抱物移动端-----#
        # 创建规划变量
        self.Tb_t1 = np.zeros([self.num, 4, 4])
        self.Tb_t2 = np.zeros([self.num, 4, 4])

        # 坐标变换求取工具坐标系在基座下的表示
        for i in range(self.num):
            self.Tb_t1[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b1),
                                                self.To_array[i, :, :]),
                                         self.To_t1)

            self.Tb_t2[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b2),
                                                self.To_array[i, :, :]),
                                         self.To_t2)

    def put_tool_robots_bass(self, Xw_o):
        # 转换为齐次矩阵
        Tw_o = np.eye(4)
        Tw_o[0:3, 0:3] = bf.euler_zyx2rot(Xw_o[3:6])
        Tw_o[0:3, 3] = Xw_o[0:3]

        # 求取基座系下工具位置
        Tb_t1 = np.dot(np.dot(nla.inv(self.Tw_b1), Tw_o), self.To_t1)
        Tb_t2 = np.dot(np.dot(nla.inv(self.Tw_b2), Tw_o), self.To_t2)
        return [Tb_t1, Tb_t2]

    def put_robots_joint_position(self, q1_guess, q2_guess):
        # 求取工具坐标基座标系的表示
        self.tool_to_robots_base()

        # 建立关节角变量
        num = self.num + self.r_num

        qq1 = np.zeros([num, self.n1])
        qq2 = np.zeros([num, self.n2])
        qq1_guess = np.copy(q1_guess)
        qq2_guess = np.copy(q2_guess)

        Tb_t1 = np.zeros([num, 4, 4])
        Tb_t2 = np.zeros([num, 4, 4])

        Tb_t1[0:self.r_num, :, :] = np.copy(self.r_Tb_t1)
        Tb_t1[self.r_num:, :, :] = np.copy(self.Tb_t1)

        Tb_t2[0:self.r_num, :, :] = np.copy(self.r_Tb_t2)
        Tb_t2[self.r_num:, :, :] = np.copy(self.Tb_t2)

        # 求取逆运动学
        for i in range(num):
            qq1[i, :] = self.kin1.iterate_ikine_limit(qq1_guess, Tb_t1[i, :, :])
            qq2[i, :] = self.kin2.iterate_ikine_limit(qq2_guess, Tb_t2[i, :, :])
            qq1_guess = qq1[i, :]
            qq2_guess = qq2[i, :]

        #增减返回段
        ms = self.ms
        m = 2*num + ms
        qq1_s = np.zeros([m, self.n1])
        qq2_s = np.zeros([m, self.n2])
        for i in range(m):
            if(i<num):
                qq1_s[i, :] = qq1[i, :]
                qq2_s[i, :] = qq2[i, :]
            elif(i < num+ms):
                qq1_s[i, :] = qq1[-1, :]
                qq2_s[i, :] = qq2[-1, :]
            else:
                qq1_s[i, :] = qq1[-1-i+(num+ms), :]
                qq2_s[i, :] = qq2[-1-i+(num+ms), :]
        return [qq1_s, qq2_s]

    #输出抓取期望力
    def put_expect_force(self, fr1, fd, fr2):
        num = 2*self.num + 2*self.r_num + self.ms

        f1 = np.zeros([num, 6])
        f2 = np.zeros([num, 6])

        for i in range(num):
            if(i<self.r_num):
                f1[i, 2] = fr1
                f2[i, 2] = fr1
            elif(i>= 2*self.num + self.r_num + self.ms):
                f1[i, 2] = fr2
                f2[i, 2] = fr2
            else:
                f1[i, 2] = fd
                f2[i, 2] = fd
        return [f1, f2]

    def put_robots_tool_position(self):
        return [self.Tb_t1, self.Tb_t2]

    def put_robots_tool_position_zyx(self):
        Xb_t1 = np.zeros(self.num, 6)
        Xb_t2 = np.zeros(self.num, 6)

        for i in range(self.num):
            Xb_t1[i, 0:3] = self.Tb_t1[i, 0:3, 3]
            Xb_t1[i, 3:6] = bf.rot2euler_zyx(self.Tb_t1[i, 0:3, 0:3])

            Xb_t2[i, 0:3] = self.Tb_t2[i, 0:3, 3]
            Xb_t2[i, 3:6] = bf.rot2euler_zyx(self.Tb_t2[i, 0:3, 0:3])

        return [Xb_t1, Xb_t2]

# ===============两个UR机械臂搬运物体类:加入手抓===================#
class RobotsHandMoveObject(object):
    def __init__(self):
        pass

    def get_robot1_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_1 = DH_0
        self.qq_min1 = qq_min
        self.qq_max1 = qq_max
        self.n1 = len(qq_min)
        self.kin1 = kin.GeneralKinematic(self.DH_0_1)

    def get_robot2_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_2 = DH_0
        self.qq_min2 = qq_min
        self.qq_max2 = qq_max
        self.n2 = len(qq_min)
        self.kin2 = kin.GeneralKinematic(self.DH_0_2)

    # 机器人基座相对世界坐标系
    def get_robots_base_to_world(self, Tw_b1, Tw_b2):
        self.Tw_b1 = np.copy(Tw_b1)
        self.Tw_b2 = np.copy(Tw_b2)

    def get_robots_base_to_world_zyx(self, Xw_b1, Xw_b2):
        self.Tw_b1 = np.eye(4)
        self.Tw_b2 = np.eye(4)

        # 转换为齐次矩阵
        self.Tw_b1[0:3, 3] = Xw_b1[0:3]
        self.Tw_b1[0:3, 0:3] = bf.euler_zyx2rot(Xw_b1[3:6])
        self.Tw_b2[0:3, 3] = Xw_b2[0:3]
        self.Tw_b2[0:3, 0:3] = bf.euler_zyx2rot(Xw_b2[3:6])

    def get_grasp_end(self, Te_t):
        self.Tt_e = np.eye(4)
        self.Tt_e[0:3, 0:3] = Te_t[0:3, 0:3].T
        self.Tt_e[0:3, 3] = -np.dot(Te_t[0:3, 0:3].T, Te_t[0:3, 3])

    # 工具坐标相对物体坐标
    def get_robots_tool_to_object(self, To_t1, To_t2):
        self.To_t1 = To_t1
        self.To_t2 = To_t2

    def get_robots_tool_to_object_zyx(self, Xo_t1, Xo_t2):
        self.To_t1 = np.eye(4)
        self.To_t2 = np.eye(4)

        # 转换为齐次矩阵
        self.To_t1[0:3, 3] = Xo_t1[0:3]
        self.To_t1[0:3, 0:3] = bf.euler_zyx2rot(Xo_t1[3:6])
        self.To_t2[0:3, 3] = Xo_t2[0:3]
        self.To_t2[0:3, 0:3] = bf.euler_zyx2rot(Xo_t2[3:6])

    # 物体在世界坐标系中的位姿
    def get_object_plan_list(self, To_array):
        '''
        :param Tb_array: [num,4,4]数组
        :return:
        '''
        # 获取规划点个数
        self.num = len(To_array)

        self.To_array = To_array

    def get_object_plan_list_zyx(self, Xo_array):
        '''
        :param Xb_array: [num,6]数组
        :return:
        '''
        # 获取规划点个数
        self.num = len(Xo_array)

        # 转换为齐次矩阵
        self.To_array = np.zeros([self.num, 4, 4])
        for i in range(self.num):
            self.To_array[i, 0:3, 0:3] = bf.euler_zyx2rot(Xo_array[i, 3:6])
            self.To_array[i, 0:3, 3] = Xo_array[i, 0:3]
            self.To_array[i, 3, 3] = 1.0

    # 获得准备段大小
    def get_ready_distance(self, l, ready_num):
        self.l = l
        self.r_num = ready_num

    def tool_to_robots_base(self):
        #加入手抓
        self.To_e1 = np.dot(self.To_t1, self.Tt_e)
        self.To_e2 = np.dot(self.To_t2, self.Tt_e)

        # #---创建准备段----#
        self.r_Tb_e1 = np.zeros([self.r_num, 4, 4])
        self.r_Tb_e2 = np.zeros([self.r_num, 4, 4])

        # 移动工具坐标系
        t = np.linspace(0, 1, self.r_num)
        l_array = self.l * np.cos(np.pi / 2 * t)

        for i in range(self.r_num):
            r_To_t1 = np.copy(self.To_t1)
            r_To_t1[0, 3] = self.To_t1[0, 3] - l_array[i]
            r_To_e1 = np.dot(r_To_t1, self.Tt_e)

            r_To_t2 = np.copy(self.To_t2)
            r_To_t2[0, 3] = self.To_t2[0, 3] + l_array[i]
            r_To_e2 = np.dot(r_To_t2, self.Tt_e)

            # 坐标变换求取工具坐标系在基座下的表示
            self.r_Tb_e1[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b1),
                                                  self.To_array[0, :, :]),
                                           r_To_e1)
            self.r_Tb_e2[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b2),
                                                  self.To_array[0, :, :]),
                                           r_To_e2)

        # ----抱物移动端-----#
        # 创建规划变量
        self.Tb_e1 = np.zeros([self.num, 4, 4])
        self.Tb_e2 = np.zeros([self.num, 4, 4])

        # 坐标变换求取工具坐标系在基座下的表示
        for i in range(self.num):
            self.Tb_e1[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b1),
                                                self.To_array[i, :, :]),
                                         self.To_e1)

            self.Tb_e2[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b2),
                                                self.To_array[i, :, :]),
                                         self.To_e2)

    def put_tool_robots_bass(self, Xw_o):
        # 转换为齐次矩阵
        Tw_o = np.eye(4)
        Tw_o[0:3, 0:3] = bf.euler_zyx2rot(Xw_o[3:6])
        Tw_o[0:3, 3] = Xw_o[0:3]

        # 求取基座系下工具位置
        Tb_t1 = np.dot(np.dot(nla.inv(self.Tw_b1), Tw_o), self.To_t1)
        Tb_t2 = np.dot(np.dot(nla.inv(self.Tw_b2), Tw_o), self.To_t2)
        return [Tb_t1, Tb_t2]

    def put_robots_joint_position(self, q1_guess, q2_guess):
        # 求取工具坐标基座标系的表示
        self.tool_to_robots_base()

        #暂停，等待抓取
        self.s_num = 100

        # 建立关节角变量
        num = self.num + self.r_num +self.s_num

        qq1 = np.zeros([num, self.n1])
        qq2 = np.zeros([num, self.n2])
        qq1_guess = np.copy(q1_guess)
        qq2_guess = np.copy(q2_guess)

        Tb_e1 = np.zeros([num, 4, 4])
        Tb_e2 = np.zeros([num, 4, 4])

        Tb_e1[0:self.r_num, :, :] = np.copy(self.r_Tb_e1)
        for i in range(self.s_num):
            Tb_e1[self.r_num + i, :, :] = np.copy(self.r_Tb_e1[-1, :, :])
        Tb_e1[self.r_num+self.s_num:, :, :] = np.copy(self.Tb_e1)

        Tb_e2[0:self.r_num, :, :] = np.copy(self.r_Tb_e2)
        for i in range(self.s_num):
            Tb_e2[self.r_num + i, :, :] = np.copy(self.r_Tb_e2[-1, :, :])
        Tb_e2[self.r_num+self.s_num:, :, :] = np.copy(self.Tb_e2)

        # 求取逆运动学
        for i in range(num):
            qq1[i, :] = self.kin1.ur_ikine(Tb_e1[i, :, :], qq1_guess)
            qq2[i, :] = self.kin2.ur_ikine(Tb_e2[i, :, :], qq2_guess)
            qq1_guess = qq1[i, :]
            qq2_guess = qq2[i, :]
        return [qq1, qq2]

    def put_robots_tool_position(self):
        return [self.Tb_t1, self.Tb_t2]

    def put_robots_tool_position_zyx(self):
        Xb_t1 = np.zeros(self.num, 6)
        Xb_t2 = np.zeros(self.num, 6)

        for i in range(self.num):
            Xb_t1[i, 0:3] = self.Tb_t1[i, 0:3, 3]
            Xb_t1[i, 3:6] = bf.rot2euler_zyx(self.Tb_t1[i, 0:3, 0:3])

            Xb_t2[i, 0:3] = self.Tb_t2[i, 0:3, 3]
            Xb_t2[i, 3:6] = bf.rot2euler_zyx(self.Tb_t2[i, 0:3, 0:3])

        return [Xb_t1, Xb_t2]

#===============机械臂运动到给定的世界坐标点=================#
class RobotMoveToWorldPos(object):
    def __init__(self):
        pass
    def get_robot_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_1 = DH_0
        self.qq_min1 = qq_min
        self.qq_max1 = qq_max
        self.n1 = len(qq_min)
        self.kin1 = kin.GeneralKinematic(self.DH_0_1)

    def get_robots_base_to_world(self, Tw_b1):
        self.Tw_b1 = Tw_b1

    def get_robots_base_to_world_zyx(self, Xw_b1):
        self.Tw_b1 = np.eye(4)

        #转换为齐次矩阵
        self.Tw_b1[0:3, 3] = Xw_b1[0:3]
        self.Tw_b1[0:3, 0:3] = bf.euler_zyx2rot(Xw_b1[3:6])

    def put_robot1_joint_position(self, Te, qq_guess):
        #求取工具坐标基座标系的表示
        Tb_t1 = np.dot(nla.inv(self.Tw_b1), Te)

        #求取关节角变量
        qq = self.kin1.ur_ikine(Tb_t1, qq_guess)
        return qq

    def put_robot1_joint_position_zyx(self, Xe, qq_guess):
        #转换为齐次坐标
        Te = np.eye(4)
        Te[0:3, 3] = Xe[0:3]
        Te[0:3, 0:3] = bf.euler_zyx2rot(Xe[3:6])

        #求取工具坐标基座标系的表示
        Tb_t1 = np.dot(nla.inv(self.Tw_b1), Te)

        #求取关节角变量
        qq = self.kin1.ur_ikine(Tb_t1, qq_guess)
        return qq

#===============机械臂基座与世界坐标系关系=================#
class RobotBaseAndWorld(object):
    Tw_b = np.eye(4)
    def __init__(self):
        pass
    def get_robot_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_1 = DH_0
        self.qq_min1 = qq_min
        self.qq_max1 = qq_max
        self.n1 = len(qq_min)
        self.kin1 = kin.GeneralKinematic(self.DH_0_1)

    def get_robots_base_to_world(self, Tw_b):
        self.Tw_b = Tw_b

    def get_robots_base_to_world_zyx(self, Xw_b):
        self.Tw_b = np.eye(4)

        #转换为齐次矩阵
        self.Tw_b[0:3, 3] = Xw_b[0:3]
        self.Tw_b[0:3, 0:3] = bf.euler_zyx2rot(Xw_b[3:6])

    def put_world_to_base(self):
        self.Tb_w = nla.inv(self.Tw_b)
        return self.Tb_w

    def put_robot_joint_position(self, Te, qq_guess):
        #求取工具坐标基座标系的表示
        Tb_t = np.dot(nla.inv(self.Tw_b), Te)

        #求取关节角变量
        qq = self.kin1.ur_ikine(Tb_t, qq_guess)
        return qq

    def put_robot_joint_position_zyx(self, Xe, qq_guess):
        #转换为齐次坐标
        Te = np.eye(4)
        Te[0:3, 3] = Xe[0:3]
        Te[0:3, 0:3] = bf.euler_zyx2rot(Xe[3:6])

        #求取工具坐标基座标系的表示
        Tb_t = np.dot(nla.inv(self.Tw_b), Te)

        #求取关节角变量
        qq = self.kin1.ur_ikine(Tb_t, qq_guess)
        return qq

    def put_base_tool(self, qr):
        Tb = self.kin1.fkine(qr)
        return Tb

    def put_base_position(self, Tb_e, qq_guess):
        return self.kin1.ur_ikine(Tb_e, qq_guess)

    def put_world_tool(self, qr):
        Tb = self.kin1.fkine(qr)
        Tw = np.dot(self.Tw_b, Tb)

        return Tw

    def put_world_tool_zyx(self, qr):
        X = np.zeros(6)
        Tb = self.kin1.fkine(qr)
        Tw = np.dot(self.Tw_b, Tb)
        X[0:3] = Tw[0:3, 3]
        X[3:6] = bf.rot2euler_zyx(Tw[0:3, 0:3])
        return X

#===============两个UR机械臂包住物体，第三个机械臂打磨===================#
class RobotsPolishObject(object):
    def __init__(self):
        pass
    def get_robot1_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_1 = DH_0
        self.qq_min1 = qq_min
        self.qq_max1 = qq_max
        self.n1 = len(qq_min)
        self.kin1 = kin.GeneralKinematic(self.DH_0_1)

    def get_robot2_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_2 = DH_0
        self.qq_min2 = qq_min
        self.qq_max2 = qq_max
        self.n2 = len(qq_min)
        self.kin2 = kin.GeneralKinematic(self.DH_0_2)

    def get_robot3_paramter(self, DH_0, qq_min, qq_max):
        self.DH_0_3 = DH_0
        self.qq_min3 = qq_min
        self.qq_max3 = qq_max
        self.n3 = len(qq_min)
        self.kin3 = kin.GeneralKinematic(self.DH_0_3)

    #机器人基座相对世界坐标系
    def get_robots_base_to_world(self, Tw_b1, Tw_b2, Tw_b3):
        self.Tw_b1 = Tw_b1
        self.Tw_b2 = Tw_b2
        self.Tw_b3 = Tw_b3

    def get_robots_base_to_world_zyx(self, Xw_b1, Xw_b2, Xw_b3):
        self.Tw_b1 = np.eye(4)
        self.Tw_b2 = np.eye(4)
        self.Tw_b3 = np.eye(4)

        #转换为齐次矩阵
        self.Tw_b1[0:3, 3] = Xw_b1[0:3]
        self.Tw_b1[0:3, 0:3] = bf.euler_zyx2rot(Xw_b1[3:6])

        self.Tw_b2[0:3, 3] = Xw_b2[0:3]
        self.Tw_b2[0:3, 0:3] = bf.euler_zyx2rot(Xw_b2[3:6])

        self.Tw_b3[0:3, 3] = Xw_b3[0:3]
        self.Tw_b3[0:3, 0:3] = bf.euler_zyx2rot(Xw_b3[3:6])

    #工具坐标相对物体坐标
    def get_robot12_tool_to_object(self, To_t1, To_t2):
        self.To_t1 = To_t1
        self.To_t2 = To_t2

    def get_robot12_tool_to_object_zyx(self,Xo_t1, Xo_t2):
        self.To_t1 =np.eye(4)
        self.To_t2 = np.eye(4)

        # 转换为齐次矩阵
        self.To_t1[0:3, 3] = Xo_t1[0:3]
        self.To_t1[0:3, 0:3] = bf.euler_zyx2rot(Xo_t1[3:6])
        self.To_t2[0:3, 3] = Xo_t2[0:3]
        self.To_t2[0:3, 0:3] = bf.euler_zyx2rot(Xo_t2[3:6])

    def get_robot3_tool_to_object(self, To_t3_array):
        self.num3 = len(To_t3_array)
        self.To_t3_array = np.copy(To_t3_array)

    def get_robot3_tool_to_object_zyx(self, Xo_t3):
        self.num3 = len(Xo_t3)
        self.To_t3_array =np.zeros([self.num3, 4, 4])

        # 转换为齐次矩阵
        for i in range(self.num3):
            self.To_t3_array[i, 0:3, 3] = Xo_t3[i, 0:3]
            self.To_t3_array[i, 0:3, 0:3] = bf.euler_zyx2rot(Xo_t3[i, 3:6])
            self.To_t3_array[i, 3, 3] = 1

    # 物体在世界坐标系中的位姿
    def get_object_plan_list(self, To_array):
        '''
        :param Tb_array: [num,4,4]数组
        :return:
        '''
        #获取规划点个数
        self.num12 = len(To_array)

        self.To_array = To_array

    def get_object_plan_list_zyx(self, Xo_array):
        '''
        :param Xb_array: [num,6]数组
        :return:
        '''
        #获取规划点个数
        self.num12 = len(Xo_array)

        #转换为齐次矩阵
        self.To_array = np.zeros([self.num12, 4, 4])
        for i in range(self.num12):
            self.To_array[i, 0:3, 0:3] = bf.euler_zyx2rot(Xo_array[i, 3:6])
            self.To_array[i, 0:3, 3] = Xo_array[i, 0:3]
            self.To_array[i, 3, 3] = 1.0

    #获得准备段大小
    def get_ready_distance(self, l, ready_num):
        self.l = l
        self.r_num = ready_num

    def tool_to_robots_base(self):
        # #---创建准备段----#
        self.r_Tb_t1 = np.zeros([self.r_num, 4, 4])
        self.r_Tb_t2 = np.zeros([self.r_num, 4, 4])
        self.r_Tb_t3 = np.zeros([self.r_num, 4, 4])

        #移动工具坐标系
        t = np.linspace(0, 1, self.r_num)
        l_array = self.l*np.cos(np.pi/2*t)

        for i in range(self.r_num):
            r_To_t1  = np.copy(self.To_t1)
            r_To_t1[0, 3] = self.To_t1[0, 3] - l_array[i]

            r_To_t2 = np.copy(self.To_t2)
            r_To_t2[0, 3] = self.To_t2[0, 3] + l_array[i]

            r_To_t3 = np.copy(self.To_t3_array[0, :, :])
            r_To_t3[2, 3] = self.To_t3_array[0, 2, 3] + l_array[i]

            # 坐标变换求取工具坐标系在基座下的表示
            self.r_Tb_t1[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b1),
                                                self.To_array[0, :, :]),
                                           r_To_t1)
            self.r_Tb_t2[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b2),
                                                self.To_array[0, :, :]),
                                           r_To_t2)
            self.r_Tb_t3[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b3),
                                              self.To_array[-1, :, :]),
                                       r_To_t3)

        #----抱物移动端-----#
        #创建规划变量
        self.Tb_t1 = np.zeros([self.num12, 4, 4])
        self.Tb_t2 = np.zeros([self.num12, 4, 4])

        #坐标变换求取工具坐标系在基座下的表示
        for i in range(self.num12):
            self.Tb_t1[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b1),
                                                self.To_array[i, :, :]),
                                         self.To_t1)
            self.Tb_t2[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b2),
                                                self.To_array[i, :, :]),
                                         self.To_t2)

        #----打磨机械臂打磨头移动----#
        self.Tb_t3 = np.zeros([self.num3, 4, 4])
        for i in range(self.num3):
            self.Tb_t3[i, :, :] = np.dot(np.dot(nla.inv(self.Tw_b3),
                                                self.To_array[-1, :, :]),
                                         self.To_t3_array[i, :, :])

    def put_tool_robots_bass(self, Xw_o):
        #转换为齐次矩阵
        Tw_o = np.eye(4)
        Tw_o[0:3, 0:3] = bf.euler_zyx2rot(Xw_o[3:6])
        Tw_o[0:3, 3] = Xw_o[0:3]

        #求取基座系下工具位置
        Tb_t1 = np.dot(np.dot(nla.inv(self.Tw_b1), Tw_o), self.To_t1)
        Tb_t2 = np.dot(np.dot(nla.inv(self.Tw_b2), Tw_o), self.To_t2)
        return [Tb_t1, Tb_t2]

    def put_robots_joint_position(self, q1_guess, q2_guess, q3_guess):
        #求取工具坐标基座标系的表示
        self.tool_to_robots_base()

        #建立关节角变量
        num12 = self.num12 +self.r_num
        num3 = self.num3 + self.r_num

        qq1 = np.zeros([num12, self.n1])
        qq2 = np.zeros([num12, self.n2])
        qq3 = np.zeros([num3, self.n3])

        qq1_guess = np.copy(q1_guess)
        qq2_guess = np.copy(q2_guess)
        qq3_guess = np.copy(q3_guess)

        Tb_t1 = np.zeros([num12, 4, 4])
        Tb_t2 = np.zeros([num12, 4, 4])
        Tb_t3 = np.zeros([num3, 4, 4])

        Tb_t1[0:self.r_num] = np.copy(self.r_Tb_t1)
        Tb_t1[self.r_num:] = np.copy(self.Tb_t1)

        Tb_t2[0:self.r_num] = np.copy(self.r_Tb_t2)
        Tb_t2[self.r_num:] = np.copy(self.Tb_t2)

        Tb_t3[0:self.r_num] = np.copy(self.r_Tb_t3)
        Tb_t3[self.r_num:] = np.copy(self.Tb_t3)

        #求取逆运动学
        for i in range(num12):
            qq1[i, :] = self.kin1.ur_ikine(Tb_t1[i, :, :], qq1_guess)
            qq2[i, :] = self.kin2.ur_ikine(Tb_t2[i, :, :], qq2_guess)
            qq1_guess = qq1[i, :]
            qq2_guess = qq2[i, :]

        for i in range(num3):
            qq3[i, :] = self.kin2.ur_ikine(Tb_t3[i, :, :], qq3_guess)
            qq3_guess = qq3[i, :]
        return [qq1, qq2, qq3]

    def put_robots_tool_position(self):
        #求取工具坐标基座标系的表示
        self.tool_to_robots_base()
        return [self.Tb_t1, self.Tb_t2, self.Tb_t3,]

    def put_robots_tool_position_zyx(self):
        #求取工具坐标基座标系的表示
        self.tool_to_robots_base()

        Xb_t1 = np.zeros([self.num12, 6])
        Xb_t2 = np.zeros([self.num12, 6])
        Xb_t3 = np.zeros([self.num3, 6])

        for i in range(self.num12):
            Xb_t1[i, 0:3] = self.Tb_t1[i, 0:3, 3]
            Xb_t1[i, 3:6] = bf.rot2euler_zyx(self.Tb_t1[i, 0:3, 0:3])

            Xb_t2[i, 0:3] = self.Tb_t2[i, 0:3, 3]
            Xb_t2[i, 3:6] = bf.rot2euler_zyx(self.Tb_t2[i, 0:3, 0:3])

        for i in range(self.num3):
            Xb_t3[i, 0:3] = self.Tb_t3[i, 0:3, 3]
            Xb_t3[i, 3:6] = bf.rot2euler_zyx(self.Tb_t3[i, 0:3, 0:3])


        return [Xb_t1, Xb_t2, Xb_t3]

    def put_polish_force(self, Fd, m = 500):
        '''
        :param Fd: 参考坐标系为工具坐标系
        :return:
        '''
        #打磨段轨迹点数
        num = self.num3 + self.r_num
        fd = np.zeros([num, 6])

        #接触点力规划
        t = np.linspace(0, 1, m)
        fd1 = np.zeros([self.num3, 6])
        if self.num3 > 2*m:
            for i in range(m):
                fd1[i, :] = Fd * np.sin(np.pi/2*t[i])
                fd1[-i-1, :] = Fd * np.sin(np.pi / 2 * t[i])
            for i in range(self.num3 - 2*m):
                fd1[i + m, :] = Fd
        else:
            t = np.linspace(0, 1, self.num3)
            for i in range(self.num3):
                fd1[i, :] = Fd * np.sin(np.pi * t[i])

        #合成轨迹
        fd[self.r_num:, :] = fd1
        return fd

# ===============多机器人经典规划===================#
class robotsMoveObject(object):
    def __init__(self):
        self.T = 0.01

    def get_robot_paramter(self, DH_0, qq_min, qq_max):
        self.n = len(qq_min)
        print "self.n:",self.n
        self.kin = kin.GeneralKinematic(DH_0, qq_min, qq_max)

    def get_robot_base_to_world(self, Tw_b):
        self.Tw_b = np.copy(Tw_b)

        #求逆
        self.Tb_w = nla.inv(self.Tw_b)

    def get_robot_base_to_world_zyx(self, Xw_b):
        self.Tw_b = np.eye(4)
        # 转换为齐次矩阵
        self.Tw_b[0:3, 3] = Xw_b[0:3]
        self.Tw_b[0:3, 0:3] = bf.euler_zyx2rot(Xw_b[3:6])
        self.Tb_w = nla.inv(self.Tw_b)

    def out_robot_pos(self, To_array, To_t_array):
        shape_o = To_array.ndim
        shape_t = To_t_array.ndim
        if(shape_o==2 and shape_t==2):
            Tb_t_array = np.dot(np.dot(self.Tb_w, To_array), To_t_array)

        elif(shape_o==3 and shape_t==2):
            num = len(To_array)
            Tb_t_array = np.zeros([num, 4, 4])
            # 坐标变换求取工具坐标系在基座下的表示
            for i in range(num):
                Tb_t_array[i, :, :] = np.dot(np.dot(self.Tb_w, To_array[i, :, :]), To_t_array)
        elif(shape_o == 2 and shape_t == 3):
            num = len(To_t_array)
            Tb_t_array = np.zeros([num, 4, 4])
            # 坐标变换求取工具坐标系在基座下的表示
            for i in range(num):
                Tb_t_array[i, :, :] = np.dot(np.dot(self.Tb_w, To_array), To_t_array[i, :, :])

        elif (shape_o == 3 and shape_t == 3):
            num = len(To_array)
            Tb_t_array = np.zeros([num, 4, 4])
            # 坐标变换求取工具坐标系在基座下的表示
            for i in range(num):
                Tb_t_array[i, :, :] = np.dot(np.dot(self.Tb_w, To_array[i, :, :]),
                                             To_t_array[i, :, :])
        else:
            print "输入参数不对！"
            return -1
        return Tb_t_array

    def out_robot_pos_zyx(self, Xo_array, Xo_t_array):
        shape_o = Xo_array.ndim()
        shape_t = Xo_t_array.ndim()
        To = np.eye(4)
        To_t = np.eye(4)
        if (shape_o == 2 and shape_t == 2):
            To[0:3, 0:3] = bf.euler_zyx2rot(Xo_array[3:6])
            To[0:3, 3] = Xo_array[0:3]
            To_t[0:3, 0:3] = bf.euler_zyx2rot(Xo_t_array[3:6])
            To_t[0:3, 3] = Xo_t_array[0:3]
            Tb_t_array = np.dot(np.dot(self.Tb_w, To), To_t)

        elif (shape_o == 3 and shape_t == 2):
            num = len(Xo_array)
            Tb_t_array = np.zeros([num, 4, 4])
            # 坐标变换求取工具坐标系在基座下的表示
            To_t[0:3, 0:3] = bf.euler_zyx2rot(Xo_t_array[3:6])
            To_t[0:3, 3] = Xo_t_array[0:3]
            for i in range(num):
                To[0:3, 0:3] = bf.euler_zyx2rot(Xo_array[i, 3:6])
                To[0:3, 3] = Xo_array[i, 0:3]
                Tb_t_array[i, :, :] = np.dot(np.dot(self.Tb_w, To), To_t)

        elif (shape_o == 2 and shape_t == 3):
            num = len(Xo_t_array)
            Tb_t_array = np.zeros([num, 4, 4])
            # 坐标变换求取工具坐标系在基座下的表示
            To[0:3, 0:3] = bf.euler_zyx2rot(Xo_array[3:6])
            To[0:3, 3] = Xo_array[0:3]
            for i in range(num):
                To_t[0:3, 0:3] = bf.euler_zyx2rot(Xo_t_array[i, 3:6])
                To_t[0:3, 3] = Xo_t_array[i, 0:3]
                Tb_t_array[i, :, :] = np.dot(np.dot(self.Tb_w, To), To_t)

        elif (shape_o == 3 and shape_t == 3):
            num = len(Xo_array)
            Tb_t_array = np.zeros([num, 4, 4])
            # 坐标变换求取工具坐标系在基座下的表示
            for i in range(num):
                To[0:3, 0:3] = bf.euler_zyx2rot(Xo_array[i, 3:6])
                To[0:3, 3] = Xo_array[i, 0:3]
                To_t[0:3, 0:3] = bf.euler_zyx2rot(Xo_t_array[i, 3:6])
                To_t[0:3, 3] = Xo_t_array[i, 0:3]
                Tb_t_array[i, :, :] = np.dot(np.dot(self.Tb_w, To), To_t)

        else:
            print "输入参数不对！"
            return -1

        return Tb_t_array

    def out_robot_joint(self, q_guess, To_array, To_t_array):
        #转换到基坐标
        qq_guess = np.copy(q_guess)
        Te_array = self.out_robot_pos(To_array, To_t_array)
        dim = Te_array.ndim
        if(dim==2):
            qq_array = self.kin.iterate_ikine_limit(qq_guess, Te_array)
        else:
            num = len(Te_array)
            qq_array = np.zeros([num, self.n])
            for i in range(num):
                qq_array[i, :] = self.kin.iterate_ikine_limit(qq_guess, Te_array[i, :, :])
                qq_guess = qq_array[i, :]
        return qq_array

    def out_robot_joint_zyx(self, q_guess, Xo_array, Xo_t_array):
        # 转换到基坐标
        qq_guess = np.copy(q_guess)
        Te_array = self.out_robot_pos_zyx(Xo_array, Xo_t_array)
        dim = Te_array.ndim
        if (dim == 2):
            qq_array = self.kin.iterate_ikine_limit(qq_guess, Te_array)
        else:
            num = len(Te_array)
            qq_array = np.zeros([num, self.n])
            for i in range(num):
                qq_array[i, :] = self.kin.iterate_ikine_limit(qq_guess, Te_array[i, :, :])
                qq_guess = qq_array[i, :]
        return qq_array

def main():
    robot1 = RobotBaseAndWorld()
    robot1.get_robot_paramter(rp.DH0_ur5, rp.q_min_ur5, rp.q_max_ur5)
    print "robot1_base", rp.robot1_base
    robot1.get_robots_base_to_world(rp.robot1_base_T)

    #世界坐标系求取正运动学
    qr = np.array([0.8225037732789904,
                   -0.9675142791212555,
                   -1.8673386213995522,
                   -0.30673975306898527,
                   -0.8225037732789904,
                   0.0])

    print "qr:", np.around(qr*180.0/np.pi, 1)

    Te1 = robot1.put_base_tool(qr)
    print "Te1_b:\n", np.around(Te1, 2)

    qq1_guess = np.array([100, -120, -120, 60, 90, 0]) * np.pi / 180.0
    qq = robot1.put_base_position(Te1, qq1_guess)

    Te2 = robot1.put_base_tool(qq)
    print "Te2_b:\n", np.around(Te2, 2)
    print "Te2-Te1:\n", np.around(Te2-Te1,2)


    Tb_e = np.array([[-1, 0, 0, -0.25],
                    [0, 0, -1, -0.512],
                    [0, -1, 0, 0.22657],
                    [0, 0, 0, 1]])

    # 获取关节角

    print "qq:", np.around(qq, 2)
    #求取正运动学


if __name__ == '__main__':
    main()
