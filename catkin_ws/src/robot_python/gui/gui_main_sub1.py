#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于GUI开发界面函数子函数1：存储力控相关函数
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020年9月20号
#系统函数

import os
import numpy as np
import time

#pyqt5函数
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

#ros相关模块
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

#绘图函数
import pyqtgraph as pg

#线程函数
import threading

#界面函数

from impedance_form1 import Ui_ImpForm1
from impedance_form2 import Ui_ImpForm2
from impedance_form3 import Ui_ImpForm3

#自定义文件
import gui_function as gf
from robot_python import ImpedanceControl as imp
from robot_python import FileOpen as fo

# ================阻抗控制窗口1:阻抗参数调试================#
class ImpWindow1(QMainWindow, Ui_ImpForm1):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0

    def __init__(self, parent=None):
        super(ImpWindow1, self).__init__(parent)
        self.T = 0.01
        self.t = 15

        self.run_flag = False  # 开始或停止标签
        self.real_flag = False
        self.armt_flag = False

        self.read_flag = False

        self.init_flag = False
        self.imp_flag = False
        self.home_flag = False

        self.pos1 = False
        self.pos2 = False

        [DH0, q_max, q_min] = gf.get_robot_parameter("armc")
        self.DH0 = DH0
        self.q_max = q_max
        self.q_min = q_min
        self.robot = "armc"

        self.sub_force_path = "/ft_sensor_topic"
        self.sub_pos_path = "/armc/joint_states"
        self.pub_path = "/armc/joint_positions_controller/command"
        self.n = 7  # 机械臂关节数

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')

        # 文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window ', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        fileMenu.addAction(openMain)

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        # 读取积分自适应阻抗参数MBKI
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_get_init.clicked.connect(self.calculation_init_point)
        self.checkBox_real.stateChanged.connect(self.real_and_arm)
        self.checkBox_arm.stateChanged.connect(self.real_and_arm)
        self.button_read.clicked.connect(self.read_paramter)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_init.clicked.connect(self.go_init)
        self.button_imp.clicked.connect(self.go_imp)
        self.button_end.clicked.connect(self.go_home)

    # ===============按钮功能模块相关函数================#
    # 采用pyqtgraph绘制曲线,添加画板
    def set_graph_ui(self):
        pg.setConfigOptions(antialias=True)  # pg全局变量设置函数，antialias=True开启曲线抗锯齿

        win1 = pg.GraphicsLayoutWidget()  # 创建pg layout，可实现数据界面布局自动管理
        win2 = pg.GraphicsLayoutWidget()

        # pg绘图窗口可以作为一个widget添加到GUI中的graph_layout，当然也可以添加到Qt其他所有的容器中
        self.horizontalLayout_1.addWidget(win1)
        self.horizontalLayout_2.addWidget(win2)

        p1 = win1.addPlot(title="joint pos")  # 添加第一个绘图窗口
        p1.setLabel('left', text='pos/rad', color='#ffffff')  # y轴设置函数
        p1.showGrid(x=True, y=True)  # 栅格设置函数
        p1.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p1.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p1.addLegend(size=(50, 30))  # 可选择是否添加legend

        p2 = win2.addPlot(title="force")  # 添加第一个绘图窗口
        p2.setLabel('left', text='force/N', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1, p2

        # 绘画关节角和关节角速度曲线

    def plot_joint(self, t1, qq, t2, f):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)
        # 绘制速度图
        self.p2.plot(t2, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t2, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t2, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t2, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t2, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t2, f[:, 5], pen='y', name='F6', clear=False)

    # 支持armt、armc四种状态切换
    def real_and_arm(self):
        self.real_flag = self.checkBox_real.isChecked()
        self.armt_flag = self.checkBox_arm.isChecked()
        if (self.real_flag):
            if (self.armt_flag):
                self.sub_force_path = "/armt/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/armt/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/armt/joint_command"
                self.lineEdit_pub_qq.setText(self.pub_path)
            else:
                self.sub_force_path = "/armc/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/all_joints_position_group_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)

        else:
            if (self.armt_flag):
                self.sub_force_path = "/robot1/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/robot1/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/robot1/armt_position_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)
            else:
                self.sub_force_path = "/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/armc/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/armc/joint_positions_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)

        # 输入参数
        robot = 'armc'
        if (self.armt_flag):
            robot = 'armt'
        [DH0, q_max, q_min] = gf.get_robot_parameter(robot)
        self.DH0 = DH0
        self.q_max = q_max
        self.q_min = q_min
        self.robot = robot

    # 计算初始位置
    def calculation_init_point(self):
        # 初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()

        # 转换单位
        qq = qq * np.pi / 180.0

        # 计算初始位置
        xx_b = gf.get_begin_point(qq, self.robot)

        # 转换到显示单位
        xx = np.copy(xx_b)  # 转化为mm显示
        xx[0:3] = xx_b[0:3] * 1000  # 转换为mm显示

        # 显示到界面
        self.lineEdit_x1.setText(str(round(xx[0], 4)))
        self.lineEdit_x2.setText(str(round(xx[1], 4)))
        self.lineEdit_x3.setText(str(round(xx[2], 4)))
        self.lineEdit_x4.setText(str(round(xx[3], 4)))
        self.lineEdit_x5.setText(str(round(xx[4], 4)))
        self.lineEdit_x6.setText(str(round(xx[5], 4)))
        msg = "初始末端位置\n" + \
              "x1:" + str(xx[0]) + "\n" + "x2:" + str(xx[1]) + \
              "\n" + "x3:" + str(xx[2]) + "\n" + "x4:" + str(xx[3]) + \
              "\n" + "x5:" + str(xx[4]) + "\n" + "x6:" + str(xx[5]) + "\n"
        self.textEdit.setText(msg)

    # 读取所有初始参数
    def read_paramter(self):
        # 初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()

        # 读取期望末端位置
        xx = np.zeros(6)
        xx[0] = self.lineEdit_x1.text()
        xx[1] = self.lineEdit_x2.text()
        xx[2] = self.lineEdit_x3.text()
        xx[3] = self.lineEdit_x4.text()
        xx[4] = self.lineEdit_x5.text()
        xx[5] = self.lineEdit_x6.text()
        # 改变为输入单位
        x_d = np.copy(xx)
        x_d[0:3] = xx[0:3] / 1000.0

        # 读取期望末端力,工具坐标系中描述
        f = np.zeros(6)
        f[0] = self.lineEdit_f1.text()
        f[1] = self.lineEdit_f2.text()
        f[2] = self.lineEdit_f3.text()
        f[3] = self.lineEdit_f4.text()
        f[4] = self.lineEdit_f5.text()
        f[5] = self.lineEdit_f6.text()

        # 获取阻抗参数
        # 读取阻抗参数
        M = np.zeros(6)
        M[0] = self.lineEdit_m1.text()
        M[1] = self.lineEdit_m2.text()
        M[2] = self.lineEdit_m3.text()
        M[3] = self.lineEdit_m4.text()
        M[4] = self.lineEdit_m5.text()
        M[5] = self.lineEdit_m6.text()

        B = np.zeros(6)
        B[0] = self.lineEdit_b1.text()
        B[1] = self.lineEdit_b2.text()
        B[2] = self.lineEdit_b3.text()
        B[3] = self.lineEdit_b4.text()
        B[4] = self.lineEdit_b5.text()
        B[5] = self.lineEdit_b6.text()

        K = np.zeros(6)
        K[0] = self.lineEdit_k1.text()
        K[1] = self.lineEdit_k2.text()
        K[2] = self.lineEdit_k3.text()
        K[3] = self.lineEdit_k4.text()
        K[4] = self.lineEdit_k5.text()
        K[5] = self.lineEdit_k6.text()

        I = np.zeros(6)
        I[0] = self.lineEdit_i1.text()
        I[1] = self.lineEdit_i2.text()
        I[2] = self.lineEdit_i3.text()
        I[3] = self.lineEdit_i4.text()
        I[4] = self.lineEdit_i5.text()
        I[5] = self.lineEdit_i6.text()

        msg_joint = "初始关节角\n" + \
                    "qq:" + "[" + str(qq[0]) + "," + str(qq[1]) + "," + \
                    str(qq[2]) + "," + str(qq[3]) + "," + str(qq[4]) + \
                    "," + str(qq[5]) + "," + str(qq[6]) + "]" + "\n"

        msg_pos = "期望末端位置\n" + \
                  "Xd:" + "[" + str(xx[0]) + "," + str(xx[1]) + "," + \
                  str(xx[2]) + "," + str(xx[3]) + "," + str(xx[4]) + \
                  "," + str(xx[5]) + "]" + "\n"

        msg_force = "期望末端力\n" + \
                    "Fd:" + "[" + str(f[0]) + "," + str(f[1]) + "," + \
                    str(f[2]) + "," + str(f[3]) + "," + str(f[4]) + \
                    "," + str(f[5]) + "]" + "\n"

        msg_imp = "阻抗参数\n" + \
                  "M:" + "[" + str(M[0]) + "," + str(M[1]) + "," + \
                  str(M[2]) + "," + str(M[3]) + "," + str(M[4]) + \
                  "," + str(M[5]) + "]" + "\n" + \
                  "B:" + "[" + str(B[0]) + "," + str(B[1]) + "," + \
                  str(B[2]) + "," + str(B[3]) + "," + str(B[4]) + \
                  "," + str(B[5]) + "]" + "\n" + \
                  "K:" + "[" + str(K[0]) + "," + str(K[1]) + "," + \
                  str(K[2]) + "," + str(K[3]) + "," + str(K[4]) + \
                  "," + str(K[5]) + "]" + "\n" + \
                  "I:" + "[" + str(I[0]) + "," + str(I[1]) + "," + \
                  str(I[2]) + "," + str(I[3]) + "," + str(I[4]) + \
                  "," + str(I[5]) + "]" + "\n"

        msg = msg_joint + msg_pos + msg_force + msg_imp
        self.textEdit.setText(msg)
        # 转化为输入单位，并保存到全局变量
        self.qq_init = np.copy(qq * np.pi / 180.0)
        self.xx_d = np.copy(x_d)
        self.f_d = np.copy(f)
        self.M = np.copy(M)
        self.B = np.copy(B)
        self.K = np.copy(K)
        self.I = np.copy(I)

        self.read_flag = True

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        # 存储数据
        self.qq_state = np.copy(qq)
        self.state_t = self.state_t + self.T
        self.state_qq_list.append(self.qq_state)
        self.state_t_list.append(self.state_t)
        # 仅记录1000个数据点
        del self.state_t_list[0]
        del self.state_qq_list[0]

    ##关节角订阅回调函数
    def force_callback(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.f_state = np.copy(f)
        self.state_f_list.append(f)
        # 仅记录1000个数据点
        del self.state_f_list[0]

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def probar_show(self):
        self.step_p = self.step_p + 1
        self.progressBar.setValue(self.step_p)
        if (self.step_p > 99):
            self.timer_p.stop()

    def stop(self):
        self.run_flag = False

    def realtime_plot(self):
        plot_t = np.array(self.state_t_list)
        plot_qq = np.array(self.state_qq_list)
        plot_f = np.array(self.state_f_list)
        self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

    def run_topic(self):
        # 读取话题地址
        self.sub_force_path = str(self.lineEdit_sub_f.text())
        self.sub_pos_path = str(self.lineEdit_sub_qq.text())
        self.pub_path = str(self.lineEdit_pub_qq.text())
        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos_path, JointState, self.joint_callback)
        rospy.Subscriber(self.sub_force_path, WrenchStamped, self.force_callback)
        self.pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"
        self.textEdit.setText(msg_tip)
        t1.start()

    def go_init(self):
        print "sss"
        if (not self.read_flag):
            msg = "未读取参数！\n"
            self.textEdit.setText(msg)
            return -1

        self.init_flag = True
        self.imp_flag = False
        self.home_flag = False

        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        # 调用规划函数
        [qq, _, _] = gf.q_joint_space_plan_time(qq_b, self.qq_init, self.T, self.t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_joint(t, qq, t, np.zeros([k, 6]))
        # 将规划好的位置定义为全局变量
        self.command_qq_init = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    def go_imp(self):
        self.init_flag = False
        self.imp_flag = True
        self.home_flag = False

        msg = "已切换到阻抗模式！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        self.init_flag = False
        self.imp_flag = False
        self.home_flag = True

        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        qq_home = np.array([0, 0, 0, 0, 0, 0, 0.0])
        # 调用规划函数
        [qq, qv, qa] = gf.q_joint_space_plan_time(qq_b, qq_home, self.T, self.t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_joint(t, qq, t, np.zeros([k, 6]))
        # 将规划好的位置定义为全局变量
        self.command_qq_home = np.copy(qq)
        msg = "运动到家点已规划！\n"
        self.textEdit.setText(msg)

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        # 提示标语
        msg = "开始下发命令！\n"
        self.textEdit.setText(msg)
        # 创建阻抗自适应阻抗实例
        imp_arm1 = imp.IIMPController_iter_vel()

        # 输入参数
        imp_arm1.get_robot_parameter(self.DH0, self.q_max, self.q_min, )
        imp_arm1.get_period(self.T)
        # 实时调整阻抗参数
        imp_arm1.get_imp_parameter(self.M, self.B, self.K, self.I)

        # 设置绘图,用Qtimer开线程处理（线程4）
        # self.timer_plot = QTimer()
        # self.timer_plot.timeout.connect(self.realtime_plot)
        # self.timer_plot.start(1000)

        # 发送关节角度
        rate = rospy.Rate(100)
        kk1 = 0
        k1 = 0
        kk2 = 1
        k2 = 0
        kk3 = 0
        k3 = 0
        if (self.init_flag):
            kk1 = len(self.command_qq_init)
            k1 = 0
        if (self.home_flag):
            kk3 = len(self.command_qq_home)
            k3 = 0

        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                break

            if (self.init_flag):
                command_data = Float64MultiArray()
                if (k1 < kk1):
                    command_data.data = self.command_qq_init[k1, 0:self.n]
                else:
                    command_data.data = self.command_qq_init[-1, 0:self.n]
                self.pub.publish(command_data)
                if (k1 == 0):
                    msg = "运动到初始位置已经开始！"
                    self.textEdit.setText(msg)
                k1 = k1 + 1

            if (self.imp_flag):
                # 读取期望位姿和关节角
                imp_arm1.get_expect_joint(self.qq_init)
                imp_arm1.get_current_joint(np.array(self.state_qq_list[-1]))
                # 读取当前关节角和力
                imp_arm1.get_expect_force(self.f_d)
                imp_arm1.get_current_force(np.array(self.state_f_list[-1]))
                # 计算修正关节角
                qr = imp_arm1.compute_imp_joint()
                # 发送数据
                command_data = Float64MultiArray()
                command_data.data = qr
                self.pub.publish(command_data)

                if (k2 == 0):
                    msg = "阻抗控制已经开始！"
                    self.textEdit.setText(msg)
                k2 = k2 + 1

            if (self.home_flag):
                command_data = Float64MultiArray()
                if (k3 < kk3):
                    command_data.data = self.command_qq_home[k3, 0:self.n]
                else:
                    break
                    command_data.data = self.command_qq_home[-1, 0:self.n]
                self.pub.publish(command_data)
                if (k3 == 0):
                    msg = "返回到家点已经开始！"
                    self.textEdit.setText(msg)
                k3 = k3 + 1
            QApplication.processEvents()

            rate.sleep()

    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================阻抗控制窗口2:阻抗轨迹控制================#
class ImpWindow2(QMainWindow, Ui_ImpForm2):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0

    def __init__(self, parent=None):
        super(ImpWindow2, self).__init__(parent)
        self.T = 0.01
        self.run_flag = True  # 开始或停止标签
        self.real_flag = False
        self.armt_flag = False

        self.init_flag = False
        self.imp_flag = False
        self.home_flag = False

        self.storage_flag = False
        self.read_pos_flag = False

        self.sub_force_path = "/ft_sensor_topic"
        self.sub_pos_path = "/armc/joint_states"
        self.pub_path = "/armc/joint_positions_controller/command"
        self.n = 7  # 机械臂关节数

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        mainMenu = menubar.addMenu('&Main')

        #文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window ', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        mainMenu.addAction(openMain)

        # -------------------文件菜单-------------------#
        # 中打开文件操作
        openFile = QAction(QIcon('exit.png'), 'Open', self)
        openFile.setShortcut('Ctrl+o')
        openFile.setStatusTip('Open new File')
        openFile.triggered.connect(self.fileOpen)
        fileMenu.addAction(openFile)

        # 文件菜单中关闭操作
        exitAction = QAction(QIcon('exit.png'), '&Exit', self)
        exitAction.setShortcut('Ctrl+q')
        exitAction.setStatusTip('Exit application')
        exitAction.triggered.connect(qApp.quit)
        fileMenu.addAction(exitAction)

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        # 读取积分自适应阻抗参数MBKI
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_init.clicked.connect(self.go_init)
        self.button_imp.clicked.connect(self.go_imp)
        self.button_end.clicked.connect(self.go_home)
        self.checkBox_gazebo.stateChanged.connect(self.real_and_arm)
        self.checkBox_UR5.stateChanged.connect(self.real_and_arm)
        self.button_read_pos.clicked.connect(self.read_pos)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_read_1.clicked.connect(self.read_force1)
        self.button_read_2 .clicked.connect(self.read_force2)

    # ===============按钮功能模块相关函数================#
    # 采用pyqtgraph绘制曲线,添加画板
    def set_graph_ui(self):
        pg.setConfigOptions(antialias=True)  # pg全局变量设置函数，antialias=True开启曲线抗锯齿

        win1 = pg.GraphicsLayoutWidget()  # 创建pg layout，可实现数据界面布局自动管理
        win2 = pg.GraphicsLayoutWidget()

        # pg绘图窗口可以作为一个widget添加到GUI中的graph_layout，当然也可以添加到Qt其他所有的容器中
        self.horizontalLayout_1.addWidget(win1)
        self.horizontalLayout_2.addWidget(win2)

        p1 = win1.addPlot(title="joint pos")  # 添加第一个绘图窗口
        p1.setLabel('left', text='pos/rad', color='#ffffff')  # y轴设置函数
        p1.showGrid(x=True, y=True)  # 栅格设置函数
        p1.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p1.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p1.addLegend(size=(50, 30))  # 可选择是否添加legend

        p2 = win2.addPlot(title="force")  # 添加第一个绘图窗口
        p2.setLabel('left', text='force/N', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1, p2

        # 绘画关节角和关节角速度曲线

    def plot_joint(self, t1, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)

    def plot_force(self, t2, f):
        # 绘制力图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p2.plot(t2, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t2, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t2, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t2, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t2, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t2, f[:, 5], pen='y', name='F6', clear=False)

    # 打开文件的地址和内容
    def fileOpen(self):
        # 打开文件操作
        path = os.path.join(os.getcwd(), '../')
        path = os.path.abspath(path)
        fname = QFileDialog.getOpenFileName(self, 'Open file', path)

        if fname[0]:
            f = open(fname[0], 'r')
            self.filedir = fname[0]
            self.lineEdit_path.setText(self.filedir)

            with f:
                data = f.read()
                self.pubdata = data
                self.textEdit.setText(data)

    #支持armt、armc四种状态切换
    def real_and_arm(self):
        self.real_flag = self.checkBox_gazebo.isChecked()
        self.armt_flag = self.checkBox_UR5.isChecked()
        if (self.real_flag):
            if(self.armt_flag):
                self.sub_force_path = "/armt/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/armt/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/armt/joint_command"
                self.lineEdit_pub_qq.setText(self.pub_path)
            else:
                self.sub_force_path = "/armc/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/all_joints_position_group_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)

        else:
            if(self.armt_flag):
                self.sub_force_path = "/robot1/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/robot1/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/robot1/armt_position_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)
            else:
                self.sub_force_path = "/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/armc/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/armc/joint_positions_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)

    def read_force1(self):
        msg = ""
        if(self.read_pos_flag):
            # 初始关节角
            f = np.zeros(6)
            f[0] = self.lineEdit_q1.text()
            f[1] = self.lineEdit_q2.text()
            f[2] = self.lineEdit_q3.text()
            f[3] = self.lineEdit_q4.text()
            f[4] = self.lineEdit_q5.text()
            f[5] = self.lineEdit_q6.text()

            msg = "期望恒力\n" + \
                        "F:" + "[" + str(f[0]) + "," + str(f[1]) + "," + \
                        str(f[2]) + "," + str(f[3]) + "," + str(f[4]) + \
                        "," + str(f[5]) + "]" + "\n"
            # 转化为输入单位，并保存到全局变量
            m = len(self.command_qq_imp)
            self.command_force = np.zeros([m, 6])
            for i in range(m):
                self.command_force[i, :] = f

        else:
            msg = "先读取位置!"
        self.textEdit.setText(msg)

    def read_force2(self):
        #读取数据
        force_path = str(self.lineEdit_data_f.text())

        self.command_force = fo.read(force_path)

        # 调用绘图函数
        k = len(self.command_force)
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_force(t, self.command_force)

        msg = "已获取末端期望力！"
        self.textEdit.setText(msg)

    def read_pos(self):
        #读取数据
        pos_path = str(self.lineEdit_data_qq.text())

        self.command_qq_imp = fo.read(pos_path)

        # 调用绘图函数
        k = len(self.command_qq_imp)
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_joint(t, self.command_qq_imp)

        msg = "位置已读取！"
        self.read_pos_flag = True
        self.textEdit.setText(msg)

    #开始存储取数据
    def storage_begin(self):
        self.storage_flag = True
        #存储变量清空
        self.storage_qq = []
        self.storage_f = []
        msg = "开始写入数据！\n"
        self.textEdit.setText(msg)

    # 开始存储取数据
    def storage_end(self):
        self.storage_flag = False
        # 存储路径
        file_path = "/home/d/catkin_ws/src/robot_bag/imp_data"
        time_str = time.strftime("%Y%m%d%H%M%S")
        pos_path = file_path + "/position_" + time_str + ".txt"
        force_path = file_path + "/force_" + time_str + ".txt"

        #转换数据类型
        qq_data = np.array(self.storage_qq)
        force_data = np.array(self.storage_f)

        #写入数据
        fo.write(qq_data, pos_path)
        fo.write(force_data, force_path)

        msg = "数据已写入：/home/d/catkin_ws/src/robot_bag/imp_data\n"
        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        # 存储数据
        if(self.storage_flag):
            self.storage_qq.append(qq)

        #绘图数据
        self.state_t = self.state_t + self.T
        self.state_qq_list.append(qq)
        self.state_t_list.append(self.state_t)
        # 仅记录1000个数据点
        del self.state_t_list[0]
        del self.state_qq_list[0]

    ##关节角订阅回调函数
    def force_callback(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        if (self.storage_flag):
            self.storage_f.append(f)

        #绘图数据
        self.state_f_list.append(f)
        # 仅记录1000个数据点
        del self.state_f_list[0]

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def probar_show(self):
        self.step_p = self.step_p + 1
        self.progressBar.setValue(self.step_p)
        if (self.step_p > 99):
            self.timer_p.stop()

    def stop(self):
        self.run_flag = False

    def realtime_plot(self):
        plot_t = np.array(self.state_t_list)
        plot_qq = np.array(self.state_qq_list)
        plot_f = np.array(self.state_f_list)
        self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

    def run_topic(self):
        # 读取时间
        self.T = float(self.lineEdit_T.text())
        self.t = float(self.lineEdit_t.text())

        msg_time = "获取时间：\n" + "周期T：" + str(self.T) + \
                   "\n规划时长：" + str(self.t) + "\n"
        #读取话题
        self.sub_force_path = str(self.lineEdit_sub_f.text())
        self.sub_pos_path = str(self.lineEdit_sub_qq.text())
        self.pub_path = str(self.lineEdit_pub_qq.text())

        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos_path, JointState, self.joint_callback)
        rospy.Subscriber(self.sub_force_path, WrenchStamped, self.force_callback)
        self.pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!\n"

        msg = msg_time + msg_tip
        self.textEdit.setText(msg)
        t1.start()

    def go_init(self):
        self.init_flag = True
        self.imp_flag = False
        self.home_flag = False

        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        # 调用规划函数
        [qq, qv, qa] = gf.q_joint_space_plan_time(qq_b, self.command_qq_imp[-1, :],
                                                  self.T, self.t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_joint(t, qq)
        # 将规划好的位置定义为全局变量
        self.command_qq_init = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    def go_imp(self):
        self.init_flag = False
        self.imp_flag = True
        self.home_flag = False

        msg = "已切换到阻抗模式！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        self.init_flag = False
        self.imp_flag = False
        self.home_flag = True

        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        qq_home = np.array([0, 0, 0, 0, 0, 0, 0.0])
        # 调用规划函数
        [qq, qv, qa] = gf.q_joint_space_plan_time(qq_b, qq_home, self.T, self.t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_joint(t, qq)
        # 将规划好的位置定义为全局变量
        self.command_qq_home = np.copy(qq)
        msg = "运动到家点已规划！\n"
        self.textEdit.setText(msg)

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        #提示标语
        msg = "开始下发命令！\n"
        self.textEdit.setText(msg)
        # 创建阻抗自适应阻抗实例
        imp_arm1 = imp.IIMPController_iter_vel()

        #获取阻抗参数
        self.M = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
        self.B = np.array([0.0, 0.0, 2000.0, 0.0, 0.0, 0.0])
        self.K = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.I = np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0])

        # 输入参数
        robot = 'armc'
        if(self.armt_flag):
            robot = 'armt'
        [DH0, q_max, q_min] = gf.get_robot_parameter(robot)
        imp_arm1.get_robot_parameter(DH0, q_max, q_min, )
        imp_arm1.get_period(self.T)
        # 实时调整阻抗参数
        imp_arm1.get_imp_parameter(self.M, self.B, self.K, self.I)

        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        # self.step_p = 0
        # self.timer_p = QTimer()
        # self.timer_p.timeout.connect(self.probar_show)
        # self.timer_p.start(100)

        # 设置绘图,用Qtimer开线程处理（线程4）
        # self.timer_plot = QTimer()
        # self.timer_plot.timeout.connect(self.realtime_plot)
        # self.timer_plot.start(1000)

        # 发送关节角度
        rate = rospy.Rate(100)
        kk = 0
        k = 0
        if(self.init_flag):
            kk = len(self.command_qq_init)
        if(self.imp_flag):
            kk = len(self.command_qq_imp)
        if(self.home_flag):
            kk = len(self.command_qq_home)

        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                # self.timer_plot.stop()
                # self.timer_p.stop()
                msg = "紧急停止！"
                self.textEdit.setText(msg)
                break
            command_data = Float64MultiArray()
            if(self.init_flag):
                if (k < kk):
                    command_data.data = self.command_qq_init[k, 0:self.n]
                else:
                    msg = "运动已完成！"
                    self.textEdit.setText(msg)
                    break
                if (k==0):
                    msg = "运动到初始位置已经开始！"
                    self.textEdit.setText(msg)
            elif(self.imp_flag):
                if(k<kk):
                    qq = self.command_qq_imp[k, 0:self.n]
                    fd = self.command_force[k, :]
                else:
                    qq = self.command_qq_imp[-1, 0:self.n]
                    fd = self.command_force[-1, :]
                # 读取期望位姿和关节角
                imp_arm1.get_expect_joint(qq)
                imp_arm1.get_current_joint(np.array(self.state_qq_list[-1]))
                # 读取当前关节角和力
                imp_arm1.get_expect_force(fd)
                imp_arm1.get_current_force(np.array(self.state_f_list[-1]))
                # 计算修正关节角
                qr = imp_arm1.compute_imp_joint()
                # 发送数据

                command_data.data = qr
                if (k==0):
                    msg = "阻抗控制已经开始！"
                    self.textEdit.setText(msg)
            else:
                if(k<kk):
                    command_data.data = self.command_qq_home[k, 0:self.n]
                else:
                    msg = "运动已完成！"
                    self.textEdit.setText(msg)
                    break
                if (k == 0):
                    msg = "返回到家点已经开始！"
                    self.textEdit.setText(msg)
            k = k + 1
            self.pub.publish(command_data)
            QApplication.processEvents()

            rate.sleep()

    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================阻抗控制窗口3:阻抗刚度评估================#
class ImpWindow3(QMainWindow, Ui_ImpForm3):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0

    def __init__(self, parent=None):
        super(ImpWindow3, self).__init__(parent)
        self.T = 0.01
        self.t = 10

        self.run_flag = False  # 开始或停止标签
        self.real_flag = False
        self.armt_flag = False

        self.read_flag = False

        self.init_flag = False
        self.imp_flag = False
        self.home_flag = False

        self.pos1 = False
        self.pos2 = False

        [DH0, q_max, q_min] = gf.get_robot_parameter("armc")
        self.DH0 = DH0
        self.q_max = q_max
        self.q_min = q_min
        self.robot = "armc"

        self.sub_force_path = "/ft_sensor_topic"
        self.sub_pos_path = "/armc/joint_states"
        self.pub_path = "/armc/joint_positions_controller/command"
        self.n = 7  # 机械臂关节数

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')

        # 文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window ', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        fileMenu.addAction(openMain)

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        # 读取积分自适应阻抗参数MBKI
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_get_init.clicked.connect(self.calculation_init_point)
        self.checkBox_real.stateChanged.connect(self.real_and_arm)
        self.checkBox_arm.stateChanged.connect(self.real_and_arm)
        self.button_read.clicked.connect(self.read_paramter)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_init.clicked.connect(self.go_init)
        self.button_imp.clicked.connect(self.go_imp)
        self.button_end.clicked.connect(self.go_home)
        self.button_read_pos1.clicked.connect(self.simple_pos1)
        self.button_read_pos2.clicked.connect(self.simple_pos2)
        self.button_stiff.clicked.connect(self.calculate_stiff)

    # ===============按钮功能模块相关函数================#
    # 采用pyqtgraph绘制曲线,添加画板
    def set_graph_ui(self):
        pg.setConfigOptions(antialias=True)  # pg全局变量设置函数，antialias=True开启曲线抗锯齿

        win1 = pg.GraphicsLayoutWidget()  # 创建pg layout，可实现数据界面布局自动管理
        win2 = pg.GraphicsLayoutWidget()

        # pg绘图窗口可以作为一个widget添加到GUI中的graph_layout，当然也可以添加到Qt其他所有的容器中
        self.horizontalLayout_1.addWidget(win1)
        self.horizontalLayout_2.addWidget(win2)

        p1 = win1.addPlot(title="joint pos")  # 添加第一个绘图窗口
        p1.setLabel('left', text='pos/rad', color='#ffffff')  # y轴设置函数
        p1.showGrid(x=True, y=True)  # 栅格设置函数
        p1.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p1.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p1.addLegend(size=(50, 30))  # 可选择是否添加legend

        p2 = win2.addPlot(title="force")  # 添加第一个绘图窗口
        p2.setLabel('left', text='force/N', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1, p2

        # 绘画关节角和关节角速度曲线

    def plot_joint(self, t1, qq, t2, f):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)
        # 绘制速度图
        self.p2.plot(t2, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t2, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t2, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t2, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t2, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t2, f[:, 5], pen='y', name='F6', clear=False)

    # 支持armt、armc四种状态切换
    def real_and_arm(self):
        self.real_flag = self.checkBox_real.isChecked()
        self.armt_flag = self.checkBox_arm.isChecked()
        if (self.real_flag):
            if (self.armt_flag):
                self.sub_force_path = "/armt/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/armt/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/armt/joint_command"
                self.lineEdit_pub_qq.setText(self.pub_path)
            else:
                self.sub_force_path = "/armc/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/armc/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/all_joints_position_group_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)

        else:
            if (self.armt_flag):
                self.sub_force_path = "/robot1/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/robot1/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/robot1/armt_position_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)
            else:
                self.sub_force_path = "/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/armc/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/armc/joint_positions_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)

        # 输入参数
        robot = 'armc'
        if (self.armt_flag):
            robot = 'armt'
        [DH0, q_max, q_min] = gf.get_robot_parameter(robot)
        # 创建等效刚度评估器
        self.stiff = imp.StiffnessEvaluation()
        self.stiff.get_robot_parameter(DH0, q_max, q_min)
        self.DH0 = DH0
        self.q_max = q_max
        self.q_min = q_min
        self.robot = robot

    #采集位置1
    def simple_pos1(self):
        if(True):
            qq = np.array(self.state_qq_list[-10:])
            self.stiff.get_pos1_joint(qq)
            F = np.array(self.state_f_list[-1])
            self.stiff.get_pos1_force(F)
            msg = "第一组数据已经采集"
            self.pos1 = True
            self.textEdit.setText(msg)
        else:
            msg = "请先控制到期望力"
            self.textEdit.setText(msg)

    # 采集位置2
    def simple_pos2(self):
        if (True):
            qq = np.array(self.state_qq_list[-10:])
            self.stiff.get_pos2_joint(qq)
            F = np.array(self.state_f_list[-1])
            self.stiff.get_pos2_force(F)
            msg = "第二组数据已经采集"
            self.pos2 = True
            self.textEdit.setText(msg)
        else:
            msg = "请先控制到期望力"
            self.textEdit.setText(msg)

    def calculate_stiff(self):
        if(not (self.pos1 and self.pos2)):
            msg = "请先采集数据"
            self.textEdit.setText(msg)
            return -1
        #计算出刚度
        K = self.stiff.compute_Stiffness()
        K = np.round(K)
        msg = "阻抗参数\n" + \
                  "K:" + "[" + str(K[0]) + "," + str(K[1]) + "," + \
                  str(K[2]) + "," + str(K[3]) + "," + str(K[4]) + \
                  "," + str(K[5]) + "]" + "\n"
        self.textEdit.setText(msg)

    # 计算初始位置
    def calculation_init_point(self):
        # 初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()

        # 转换单位
        qq = qq * np.pi / 180.0

        # 计算初始位置
        xx_b = gf.get_begin_point(qq, self.robot)

        # 转换到显示单位
        xx = np.copy(xx_b)  # 转化为mm显示
        xx[0:3] = xx_b[0:3] * 1000  # 转换为mm显示

        # 显示到界面
        self.lineEdit_x1.setText(str(round(xx[0], 4)))
        self.lineEdit_x2.setText(str(round(xx[1], 4)))
        self.lineEdit_x3.setText(str(round(xx[2], 4)))
        self.lineEdit_x4.setText(str(round(xx[3], 4)))
        self.lineEdit_x5.setText(str(round(xx[4], 4)))
        self.lineEdit_x6.setText(str(round(xx[5], 4)))
        msg = "初始末端位置\n" + \
              "x1:" + str(xx[0]) + "\n" + "x2:" + str(xx[1]) + \
              "\n" + "x3:" + str(xx[2]) + "\n" + "x4:" + str(xx[3]) + \
              "\n" + "x5:" + str(xx[4]) + "\n" + "x6:" + str(xx[5]) + "\n"
        self.textEdit.setText(msg)

    # 读取所有初始参数
    def read_paramter(self):
        # 初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()

        # 读取期望末端位置
        xx = np.zeros(6)
        xx[0] = self.lineEdit_x1.text()
        xx[1] = self.lineEdit_x2.text()
        xx[2] = self.lineEdit_x3.text()
        xx[3] = self.lineEdit_x4.text()
        xx[4] = self.lineEdit_x5.text()
        xx[5] = self.lineEdit_x6.text()
        # 改变为输入单位
        x_d = np.copy(xx)
        x_d[0:3] = xx[0:3] / 1000.0

        # 读取期望末端力,工具坐标系中描述
        f = np.zeros(6)
        f[0] = self.lineEdit_f1.text()
        f[1] = self.lineEdit_f2.text()
        f[2] = self.lineEdit_f3.text()
        f[3] = self.lineEdit_f4.text()
        f[4] = self.lineEdit_f5.text()
        f[5] = self.lineEdit_f6.text()

        #获取阻抗参数
        M = np.zeros(6)
        B = np.zeros(6)
        K = np.zeros(6)
        I = np.zeros(6)

        M[2] = 1.0
        B[2] = 200.0

        msg_joint = "初始关节角\n" + \
                    "qq:" + "[" + str(qq[0]) + "," + str(qq[1]) + "," + \
                    str(qq[2]) + "," + str(qq[3]) + "," + str(qq[4]) + \
                    "," + str(qq[5]) + "," + str(qq[6]) + "]" + "\n"

        msg_pos = "期望末端位置\n" + \
                  "Xd:" + "[" + str(xx[0]) + "," + str(xx[1]) + "," + \
                  str(xx[2]) + "," + str(xx[3]) + "," + str(xx[4]) + \
                  "," + str(xx[5]) + "]" + "\n"

        msg_force = "期望末端力\n" + \
                    "Fd:" + "[" + str(f[0]) + "," + str(f[1]) + "," + \
                    str(f[2]) + "," + str(f[3]) + "," + str(f[4]) + \
                    "," + str(f[5]) + "]" + "\n"

        msg_imp = "阻抗参数\n" + \
                  "M:" + "[" + str(M[0]) + "," + str(M[1]) + "," + \
                  str(M[2]) + "," + str(M[3]) + "," + str(M[4]) + \
                  "," + str(M[5]) + "]" + "\n" + \
                  "B:" + "[" + str(B[0]) + "," + str(B[1]) + "," + \
                  str(B[2]) + "," + str(B[3]) + "," + str(B[4]) + \
                  "," + str(B[5]) + "]" + "\n" + \
                  "K:" + "[" + str(K[0]) + "," + str(K[1]) + "," + \
                  str(K[2]) + "," + str(K[3]) + "," + str(K[4]) + \
                  "," + str(K[5]) + "]" + "\n" + \
                  "I:" + "[" + str(I[0]) + "," + str(I[1]) + "," + \
                  str(I[2]) + "," + str(I[3]) + "," + str(I[4]) + \
                  "," + str(I[5]) + "]" + "\n"

        msg = msg_joint + msg_pos + msg_force + msg_imp
        self.textEdit.setText(msg)
        # 转化为输入单位，并保存到全局变量
        self.qq_init = np.copy(qq * np.pi / 180.0)
        self.xx_d = np.copy(x_d)
        self.f_d = np.copy(f)
        self.M = np.copy(M)
        self.B = np.copy(B)
        self.K = np.copy(K)
        self.I = np.copy(I)

        self.read_flag = True

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        # 存储数据
        self.qq_state = np.copy(qq)
        self.state_t = self.state_t + self.T
        self.state_qq_list.append(self.qq_state)
        self.state_t_list.append(self.state_t)
        # 仅记录1000个数据点
        del self.state_t_list[0]
        del self.state_qq_list[0]

    ##关节角订阅回调函数
    def force_callback(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.f_state = np.copy(f)
        self.state_f_list.append(f)
        # 仅记录1000个数据点
        del self.state_f_list[0]

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def probar_show(self):
        self.step_p = self.step_p + 1
        self.progressBar.setValue(self.step_p)
        if (self.step_p > 99):
            self.timer_p.stop()

    def stop(self):
        self.run_flag = False

    def realtime_plot(self):
        plot_t = np.array(self.state_t_list)
        plot_qq = np.array(self.state_qq_list)
        plot_f = np.array(self.state_f_list)
        self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

    def run_topic(self):
        # 读取话题地址
        self.sub_force_path = str(self.lineEdit_sub_f.text())
        self.sub_pos_path = str(self.lineEdit_sub_qq.text())
        self.pub_path = str(self.lineEdit_pub_qq.text())
        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos_path, JointState, self.joint_callback)
        rospy.Subscriber(self.sub_force_path, WrenchStamped, self.force_callback)
        self.pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"
        self.textEdit.setText(msg_tip)
        t1.start()

    def go_init(self):
        print "sss"
        if(not self.read_flag):
            msg = "未读取参数！\n"
            self.textEdit.setText(msg)
            return -1

        self.init_flag = True
        self.imp_flag = False
        self.home_flag = False

        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        # 调用规划函数
        [qq, _, _] = gf.q_joint_space_plan_time(qq_b,  self.qq_init, self.T, self.t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_joint(t, qq, t, np.zeros([k, 6]))
        # 将规划好的位置定义为全局变量
        self.command_qq_init = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    def go_imp(self):
        self.init_flag = False
        self.imp_flag = True
        self.home_flag = False

        msg = "已切换到阻抗模式！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        self.init_flag = False
        self.imp_flag = False
        self.home_flag = True

        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        qq_home = np.array([0, 0, 0, 0, 0, 0, 0.0])
        # 调用规划函数
        [qq, qv, qa] = gf.q_joint_space_plan_time(qq_b, qq_home, self.T, self.t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_joint(t, qq, t, np.zeros([k, 6]))
        # 将规划好的位置定义为全局变量
        self.command_qq_home = np.copy(qq)
        msg = "运动到家点已规划！\n"
        self.textEdit.setText(msg)

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        # 提示标语
        msg = "开始下发命令！\n"
        self.textEdit.setText(msg)
        # 创建阻抗自适应阻抗实例
        imp_arm1 = imp.IIMPController_iter_vel()

        # 获取阻抗参数
        self.M = np.array([0.0, 0.0, 5.0, 0.0, 0.0, 0.0])
        self.B = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0])
        self.K = np.array([0.0, 0.0, 0, 0.0, 0.0, 0.0])
        self.I = np.array([0.0, 0.0, 0, 0.0, 0.0, 0.0])

        # 输入参数
        imp_arm1.get_robot_parameter(self.DH0, self.q_max, self.q_min, )
        imp_arm1.get_period(self.T)
        # 实时调整阻抗参数
        imp_arm1.get_imp_parameter(self.M, self.B, self.K, self.I)

        # 设置绘图,用Qtimer开线程处理（线程4）
        # self.timer_plot = QTimer()
        # self.timer_plot.timeout.connect(self.realtime_plot)
        # self.timer_plot.start(1000)

        # 发送关节角度
        rate = rospy.Rate(100)
        kk1 = 0
        k1 = 0
        kk2 = 1
        k2 = 0
        kk3 = 0
        k3 = 0
        if (self.init_flag):
            kk1 = len(self.command_qq_init)
            k1 = 0
        if (self.home_flag):
            kk3 = len(self.command_qq_home)
            k3 = 0

        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                break

            if (self.init_flag):
                command_data = Float64MultiArray()
                if (k1 < kk1):
                    command_data.data = self.command_qq_init[k1, 0:self.n]
                else:
                    command_data.data = self.command_qq_init[-1, 0:self.n]
                self.pub.publish(command_data)
                if (k1 == 0):
                    msg = "运动到初始位置已经开始！"
                    self.textEdit.setText(msg)
                k1 = k1 + 1

            if (self.imp_flag):
                # 读取期望位姿和关节角
                imp_arm1.get_expect_joint(self.qq_init)
                imp_arm1.get_current_joint(np.array(self.state_qq_list[-1]))
                # 读取当前关节角和力
                imp_arm1.get_expect_force(self.f_d)
                imp_arm1.get_current_force(np.array(self.state_f_list[-1]))
                # 计算修正关节角
                qr = imp_arm1.compute_imp_joint()
                # 发送数据
                command_data = Float64MultiArray()
                command_data.data = qr
                self.pub.publish(command_data)

                if (k2 == 0):
                    msg = "阻抗控制已经开始！"
                    self.textEdit.setText(msg)
                k2 = k2 + 1

            if (self.home_flag):
                command_data = Float64MultiArray()
                if (k3 < kk3):
                    command_data.data = self.command_qq_home[k3, 0:self.n]
                else:
                    command_data.data = self.command_qq_home[-1, 0:self.n]
                self.pub.publish(command_data)
                if (k3 == 0):
                    msg = "返回到家点已经开始！"
                    self.textEdit.setText(msg)
                k3 = k3 + 1
            QApplication.processEvents()

            rate.sleep()

    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()
