#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于GUI开发界面函数子函数：多臂函数相关
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020年9月20号
#系统函数
from gui_main import *
import sys
import os
import numpy as np
import time

#pyqt5函数
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

#ros相关模块
import rospy
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

#绘图函数
import pyqtgraph as pg

#线程函数
import threading

#界面函数
from gui_main import *

from urs_form1 import Ui_UrsForm1
from urs_form2 import Ui_UrsForm2
from urs_form3 import Ui_UrsForm3
from urs_hand_form1 import Ui_UrsHandForm1
from armct_form1 import Ui_ArmctForm1
from armct_form2 import Ui_ArmctForm2
from armct_form3 import Ui_ArmctForm3
from armctr_form1 import Ui_ArmctrForm1

from technology_form1 import Ui_TechnologyForm1

#自定义文件
import gui_function as gf
from robot_python import ImpedanceControl as imp
from robot_python import FileOpen as fo

#***********************************子窗口***************************************#
# ================三个UR5协同控制================#
class UrsWindow1(QMainWindow, Ui_UrsForm1):
    #建立全局变量
    state_qq_list1 = list(np.zeros([1000, 6]))
    state_f_list1 = list(np.zeros([1000, 6]))
    state_qq_list2 = list(np.zeros([1000, 6]))
    state_f_list2 = list(np.zeros([1000, 6]))
    state_qq_list3 = list(np.zeros([1000, 6]))
    state_f_list3 = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0.0

    def __init__(self, parent=None):
        super(UrsWindow1, self).__init__(parent)
        self.T = 0.01
        self.run_flag = True # 开始或停止标签
        self.real_flag = False
        self.gazebo_flag = False

        self.robot1_flag = False
        self.robot2_flag = False
        self.robot3_flag = False
        self.all_flag = False

        self.robot1_2_flag = False
        self.robot2_2_flag = False
        self.robot3_2_flag = False

        self.state_qq1 = np.zeros(6)
        self.state_qq2 = np.zeros(6)
        self.state_qq3 = np.zeros(6)

        self.sub_force1_path = "/robot1/ft_sensor_topic"
        self.sub_pos1_path = "/robot1/joint_states"
        self.pub1_path = "/robot1/ur5_position_controller/command"
        self.sub_force2_path = "/robot2/ft_sensor_topic"
        self.sub_pos2_path = "/robot2/joint_states"
        self.pub2_path = "/robot2/ur5_position_controller/command"
        self.sub_force3_path = "/robot3/ft_sensor_topic"
        self.sub_pos3_path = "/robot3/joint_states"
        self.pub3_path = "/robot3/ur5_position_controller/command"

        self.n = 6  # 机械臂关节数
        self.qq_wish_pos = np.zeros([3, 6])

        self.setupUi(self)
        self.initUI()
    def initUI(self):
        # ======================菜单栏功能模块=======================#
        #创建菜单
        menubar = self.menuBar()
        mainMenu = menubar.addMenu('&Main')

        # 文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window ', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        mainMenu.addAction(openMain)

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_read.clicked.connect(self.read_wish_pos)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_refresh.clicked.connect(self.refresh_radioButton)
        self.button_plan.clicked.connect(self.plan)
        self.button_home.clicked.connect(self.home)
        self.button_init.clicked.connect(self.init)

    #===============按钮功能模块相关函数================#
    #采用pyqtgraph绘制曲线,添加画板
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
        return p1,p2

    #绘画关节角和关节角速度曲线
    def plot_joint(self, t1, qq, t2, f):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)

        # 绘制速度图
        self.p2.plot(t2, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t2, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t2, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t2, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t2, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t2, f[:, 5], pen='y', name='F6', clear=False)

    #刷新按钮
    def refresh_radioButton(self):
        self.robot1_flag = self.radioButton_robot1.isChecked()
        self.robot2_flag = self.radioButton_robot2.isChecked()
        self.robot3_flag = self.radioButton_robot3.isChecked()
        self.all_flag = self.radioButton_all.isChecked()

        self.robot1_2_flag = self.radioButton_robot1_2.isChecked()
        self.robot2_2_flag = self.radioButton_robot2_2.isChecked()
        self.robot3_2_flag = self.radioButton_robot3_2.isChecked()

        self.real_flag = self.radioButton_real.isChecked()
        self.gazebo_flag = self.radioButton_gazebo.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

    #读取给定关节角度
    def read_wish_pos(self):
        qq = np.zeros(6)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()

        msg = ""
        if(self.robot1_flag or self.all_flag):
            msg_pos1 = "robot1规划目标点:\n" + \
                       "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) +\
                       "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) +\
                       "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                       "\n"
            self.qq_wish_pos[0, :] = np.copy(qq*np.pi/180.0)
            msg = msg + msg_pos1

        if (self.robot2_flag or self.all_flag):
            msg_pos = "robot2规划目标点:\n" + \
                       "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                       "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                       "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                       "\n"
            self.qq_wish_pos[1, :] = np.copy(qq * np.pi / 180.0)
            msg = msg + msg_pos

        if (self.robot3_flag or self.all_flag):
            msg_pos = "robot3规划目标点:\n" + \
                       "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                       "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                       "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                       "\n"
            self.qq_wish_pos[2, :] = np.copy(qq * np.pi / 180.0)
            msg = msg + msg_pos

        if (not (self.robot1_flag or self.robot2_flag or self.robot3_flag or self.all_flag)):
            msg = "没有选中机械臂！！！"

        self.textEdit.setText(msg)

    #给定初始位置
    def init(self):
        msg = ""
        if (self.robot1_flag or self.all_flag):
            qq = np.array([-90, -120, -120, 60, 90, 0])
            self.lineEdit_q1.setText(str(qq[0]))
            self.lineEdit_q2.setText(str(qq[1]))
            self.lineEdit_q3.setText(str(qq[2]))
            self.lineEdit_q4.setText(str(qq[3]))
            self.lineEdit_q5.setText(str(qq[4]))
            self.lineEdit_q6.setText(str(qq[5]))

        if (self.robot2_flag or self.all_flag):
            qq = np.array([-90, -60, 120, 120, -90, 0])
            self.lineEdit_q1.setText(str(qq[0]))
            self.lineEdit_q2.setText(str(qq[1]))
            self.lineEdit_q3.setText(str(qq[2]))
            self.lineEdit_q4.setText(str(qq[3]))
            self.lineEdit_q5.setText(str(qq[4]))
            self.lineEdit_q6.setText(str(qq[5]))

        if (self.robot3_flag or self.all_flag):
            qq = np.array([90, -135, 135, 180, -90, 0])
            self.lineEdit_q1.setText(str(qq[0]))
            self.lineEdit_q2.setText(str(qq[1]))
            self.lineEdit_q3.setText(str(qq[2]))
            self.lineEdit_q4.setText(str(qq[3]))
            self.lineEdit_q5.setText(str(qq[4]))
            self.lineEdit_q6.setText(str(qq[5]))

        if (not (self.robot1_flag or self.robot2_flag or self.robot3_flag or self.all_flag)):
            msg = "没有选中机械臂！\n"

    # 给定家点位置
    def home(self):
        qq = np.array([0, -90, 0, -90, 0, 0])
        self.lineEdit_q1.setText(str(qq[0]))
        self.lineEdit_q2.setText(str(qq[1]))
        self.lineEdit_q3.setText(str(qq[2]))
        self.lineEdit_q4.setText(str(qq[3]))
        self.lineEdit_q5.setText(str(qq[4]))
        self.lineEdit_q6.setText(str(qq[5]))

    #规划函数
    def plan(self):
        msg = ""
        if (self.robot1_flag or self.all_flag):
            qq_b1 = np.array(self.state_qq_list1[-2])
            #调用规划函数
            [qq,qv,qa] = gf.q_joint_space_plan_time(qq_b1, self.qq_wish_pos[0, :], t=30)
            #调用绘图函数
            k =len(qq[:, 0])
            t = np.linspace(0, self.T*(k-1), k)
            #绘制关节角位置速度图
            self.plot_joint(t, qq, t, np.zeros([k, 6]))
            #将规划好的位置定义为全局变量
            self.command_qq_list1 = np.copy(qq)
            msg1 = "robot1已规划！\n"
            msg = msg + msg1

        if (self.robot2_flag or self.all_flag):
            qq_b2 = np.array(self.state_qq_list2[-2])
            #调用规划函数
            [qq,qv,qa] = gf.q_joint_space_plan_time(qq_b2, self.qq_wish_pos[1, :])
            #调用绘图函数
            k =len(qq[:, 0])
            t = np.linspace(0, self.T*(k-1), k)
            #绘制关节角位置速度图
            self.plot_joint(t, qq, t, np.zeros([k, 6]))
            #将规划好的位置定义为全局变量
            self.command_qq_list2 = np.copy(qq)
            msg2 = "robot2已规划！\n"
            msg = msg + msg2

        if (self.robot3_flag or self.all_flag):
            qq_b3 = np.array(self.state_qq_list3[-2])
            #调用规划函数
            [qq,qv,qa] = gf.q_joint_space_plan_time(qq_b3, self.qq_wish_pos[2, :])
            #调用绘图函数
            k =len(qq[:, 0])
            t = np.linspace(0, self.T*(k-1), k)
            #绘制关节角位置速度图
            self.plot_joint(t, qq, t, np.zeros([k, 6]))
            #将规划好的位置定义为全局变量
            self.command_qq_list3 = np.copy(qq)
            msg3 = "robot3已规划！\n"
            msg = msg + msg3

        if (not (self.robot1_flag or self.robot2_flag or self.robot3_flag or self.all_flag)):
            msg = "没有选中机械臂！\n"

        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback1(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        qq_r = np.copy(qq)
        qq_r[0] = qq[2]
        qq_r[2] = qq[0]
        self.state_t = self.state_t + self.T
        self.state_qq_list1.append(qq_r)
        self.state_qq1 = qq_r
        self.state_t_list.append(self.state_t)
        # 仅记录100个数据点
        del self.state_t_list[0]
        del self.state_qq_list1[0]

    def joint_callback2(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        qq_r = np.copy(qq)
        qq_r[0] = qq[2]
        qq_r[2] = qq[0]
        self.state_qq_list2.append(qq_r)
        self.state_qq2 = qq_r
        # 仅记录100个数据点
        del self.state_qq_list2[0]

    def joint_callback3(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        qq_r = np.copy(qq)
        qq_r[0] = qq[2]
        qq_r[2] = qq[0]
        self.state_qq_list3.append(qq_r)
        self.state_qq3 = qq_r
        # 仅记录100个数据点
        del self.state_qq_list3[0]

    ##关节角订阅回调函数
    def force_callback1(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list1.append(f)
        # 仅记录1000个数据点
        del self.state_f_list1[0]

    def force_callback2(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list2.append(f)
        # 仅记录1000个数据点
        del self.state_f_list2[0]

    def force_callback3(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list3.append(f)
        # 仅记录1000个数据点
        del self.state_f_list3[0]

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def probar_show(self):
        self.step_p = self.step_p + 1
        self.progressBar.setValue(self.step_p)
        if(self.step_p>99):
            self.timer_p.stop()

    def stop(self):
        self.run_flag = False

    def realtime_plot(self):
        if(self.robot1_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list1)
            plot_f = np.array(self.state_f_list1)
            self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

        if (self.robot2_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list2)
            plot_f = np.array(self.state_f_list2)
            self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

        if (self.robot3_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list3)
            plot_f = np.array(self.state_f_list3)
            self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

    def run_topic(self):
        #读取时间
        self.T = float(self.lineEdit_T.text())
        self.t = float(self.lineEdit_t.text())

        msg_time = "获取时间：\n" + "周期T：" + str(self.T) + \
                   "\n规划时长：" + str(self.t) + "\n"

        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos1_path, JointState, self.joint_callback1)
        rospy.Subscriber(self.sub_force1_path, WrenchStamped, self.force_callback1)
        self.pub1 = rospy.Publisher(self.pub1_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos2_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force2_path, WrenchStamped, self.force_callback2)
        self.pub2 = rospy.Publisher(self.pub2_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos3_path, JointState, self.joint_callback3)
        rospy.Subscriber(self.sub_force3_path, WrenchStamped, self.force_callback3)
        self.pub3 = rospy.Publisher(self.pub3_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"

        msg = msg_time + msg_tip
        self.textEdit.setText(msg)
        t1.start()

    def begin_function(self):
        #运行标签启动
        self.run_flag = True

        #求取数据长度
        if(self.robot1_flag):
            kk = len(self.command_qq_list1)
        if (self.robot2_flag):
            kk = len(self.command_qq_list2)
        if (self.robot3_flag):
            kk = len(self.command_qq_list3)
        if (self.all_flag):
            kk = len(self.command_qq_list1)
        #进度条显示时间间隔
        show_time = int(kk*self.T*10)

        #设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(show_time)

        #设置绘图,用Qtimer开线程处理（线程4）
        self.timer_plot = QTimer()
        self.timer_plot.timeout.connect(self.realtime_plot)
        self.timer_plot.start(1000)

        #发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            #检测是否启动急停
            if(not self.run_flag):
                self.timer_p.stop()
                self.timer_plot.stop()
                break
            #发送数据
            msg = "已发送机器人：\n"
            if(self.robot1_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = self.command_qq_list1[k, 0:self.n]
                else:
                    command_data.data = self.command_qq_list1[-1, 0:self.n]
                self.pub1.publish(command_data)
                if(k==0):
                    msg = msg + "robot1已发送！\n"
            if (self.robot2_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = self.command_qq_list2[k, 0:self.n]
                else:
                    command_data.data = self.command_qq_list2[-1, 0:self.n]
                self.pub2.publish(command_data)
                if (k == 0):
                    msg = msg + "robot2已发送！\n"
            if (self.robot3_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = self.command_qq_list3[k, 0:self.n]
                else:
                    command_data.data = self.command_qq_list3[-1, 0:self.n]
                self.pub3.publish(command_data)
                if (k == 0):
                    msg = msg + "robot3已发送！\n"

            if (not (self.robot1_flag or self.robot2_flag or self.robot3_flag or self.all_flag)):
                msg = "没有选中机械臂！\n"

            self.textEdit.setText(msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    # ===============窗口跳转函数================#
    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================三个UR5协同控制:双臂位置模式搬运物体================#
class UrsWindow2(QMainWindow, Ui_UrsForm2):
    # 建立全局变量
    state_qq_list1 = list(np.zeros([1000, 6]))
    state_f_list1 = list(np.zeros([1000, 6]))
    state_qq_list2 = list(np.zeros([1000, 6]))
    state_f_list2 = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0.0

    def __init__(self, parent=None):
        super(UrsWindow2, self).__init__(parent)
        self.T = 0.01
        self.run_flag = True  # 开始或停止标签
        self.real_flag = False
        self.gazebo_flag = False

        self.robot1_flag = False
        self.robot2_flag = False
        self.all_flag = True

        self.robot1_2_flag = True
        self.robot2_2_flag = False

        self.init_flag = False
        self.move_flag = False
        self.home_flag = False

        self.state_qq1 = np.zeros(6)
        self.state_qq2 = np.zeros(6)

        self.sub_force1_path = "/robot1/ft_sensor_topic"
        self.sub_pos1_path = "/robot1/joint_states"
        self.pub1_path = "/robot1/ur5_position_controller/command"
        self.sub_force2_path = "/robot2/ft_sensor_topic"
        self.sub_pos2_path = "/robot2/joint_states"
        self.pub2_path = "/robot2/ur5_position_controller/command"

        self.n = 6  # 机械臂关节数

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        mainMenu = menubar.addMenu('&Main')

        # 文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window ', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        mainMenu.addAction(openMain)

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_read_data.clicked.connect(self.read_pos)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_refresh.clicked.connect(self.refresh_radioButton)
        self.button_work.clicked.connect(self.go_move)
        self.button_init_plan.clicked.connect(self.go_init)
        self.button_end_plan.clicked.connect(self.go_home)

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

    # 绘画关节角和关节角速度曲线
    def plot_force(self, t2, f):
        # 绘制力图
        self.p2.plot(t2, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t2, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t2, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t2, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t2, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t2, f[:, 5], pen='y', name='F6', clear=False)

    # 刷新按钮
    def refresh_radioButton(self):
        self.robot1_flag = self.radioButton_robot1.isChecked()
        self.robot2_flag = self.radioButton_robot2.isChecked()
        self.all_flag = self.radioButton_all.isChecked()

        self.robot1_2_flag = self.radioButton_robot1_2.isChecked()
        self.robot2_2_flag = self.radioButton_robot2_2.isChecked()

        self.real_flag = self.radioButton_real.isChecked()
        self.gazebo_flag = self.radioButton_gazebo.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

    def go_init(self):
        self.init_flag = True
        self.move_flag = False
        self.home_flag = False

        #先回到零位
        qq_init = np.array([0, -90, 0, -90, 0, 0.0])*np.pi/180

        if(self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])

            # 调用规划函数
            [qq1_1, qv, qa] = gf.q_joint_space_plan_time(qq_b1, qq_init, self.T, self.t)
            [qq1_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, self.robot1_command_qq[0, :],
                                                      self.T, self.t)
            # 调用绘图函数
            k1_1 = len(qq1_1)
            k1_2 = len(qq1_2)
            k1 = k1_1 + k1_2
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            qq1 = np.zeros([k1, self.n])
            qq1[:k1_1, :] = qq1_1
            qq1[k1_1:, :] = qq1_2

            # 绘制关节角位置图
            self.plot_joint(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_init = np.copy(qq1)

        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2_1, qv, qa] = gf.q_joint_space_plan_time(qq_b2, qq_init, self.T, self.t)
            [qq2_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, self.robot2_command_qq[0, :],
                                                      self.T, self.t)
            # 调用绘图函数
            k2_1 = len(qq2_1)
            k2_2 = len(qq2_2)
            k2 = k2_1 + k2_2
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            qq2 = np.zeros([k2, self.n])
            qq2[:k2_1, :] = qq2_1
            qq2[k2_1:, :] = qq2_2
            # 绘制关节角位置图
            self.plot_force(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_init = np.copy(qq2)

        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    def go_move(self):
        self.init_flag = False
        self.move_flag = True
        self.home_flag = False

        msg = "已切换到搬运模式！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        self.init_flag = False
        self.move_flag = False
        self.home_flag = True

        # 获得规划终点
        qq_home = np.array([0, 0, 0, 0, 0, 0, 0.0])
        qq_init = np.array([0, -90, 0, -90, 0, 0.0]) * np.pi / 180

        # 调用规划函数
        if (self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq1_1, qv, qa] = gf.q_joint_space_plan_time(qq_b1, qq_init, self.T, self.t)
            [qq1_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, qq_home, self.T, self.t)
            # 调用绘图函数
            k1_1 = len(qq1_1)
            k1_2 = len(qq1_2)
            k1 = k1_1 + k1_2
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            qq1 = np.zeros([k1, self.n])
            qq1[:k1_1, :] = qq1_1
            qq1[k1_1:, :] = qq1_2
            # 绘制关节角位置图
            self.plot_joint(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_home = np.copy(qq1)

        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2_1, qv, qa] = gf.q_joint_space_plan_time(qq_b2, qq_init, self.T, self.t)
            [qq2_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, qq_home, self.T, self.t)
            # 调用绘图函数
            k2_1 = len(qq2_1)
            k2_2 = len(qq2_2)
            k2 = k2_1 + k2_2
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            qq2 = np.zeros([k2, self.n])
            qq2[:k2_1, :] = qq2_1
            qq2[k2_1:, :] = qq2_2
            # 绘制关节角位置图
            self.plot_force(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_home = np.copy(qq2)

        msg = "运动到家点已规划！\n"
        self.textEdit.setText(msg)

    #读取数据
    def read_pos(self):
        #读取数据
        if(self.robot1_flag or self.all_flag):
            pos_path1 = str(self.lineEdit_data_1.text())
            self.robot1_command_qq = fo.read(pos_path1)
            # 调用绘图函数
            k = len(self.robot1_command_qq)
            t = np.linspace(0, self.T * (k - 1), k)
            # 绘制关节角位置速度图
            self.plot_joint(t, self.robot1_command_qq)

        if(self.robot2_flag or self.all_flag):
            pos_path2 = str(self.lineEdit_data_2.text())
            self.robot2_command_qq = fo.read(pos_path2)
            # 调用绘图函数
            k = len(self.robot2_command_qq)
            t = np.linspace(0, self.T * (k - 1), k)
            # 绘制关节角位置速度图
            self.plot_force(t, self.robot2_command_qq)



        msg = "位置已读取！"
        self.read_pos_flag = True
        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback1(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        #反馈关节有问题，改
        qq_r = np.copy(qq)
        qq_r[0] = qq[2]
        qq_r[2] = qq[0]
        self.state_t = self.state_t + self.T
        self.state_qq_list1.append(qq_r)
        self.state_qq1 = qq_r
        self.state_t_list.append(self.state_t)
        # 仅记录100个数据点
        del self.state_t_list[0]
        del self.state_qq_list1[0]

    def joint_callback2(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]
        qq_r = np.copy(qq)
        qq_r[0] = qq[2]
        qq_r[2] = qq[0]
        self.state_qq_list2.append(qq_r)
        self.state_qq2 = qq_r
        # 仅记录100个数据点
        del self.state_qq_list2[0]

    ##关节角订阅回调函数
    def force_callback1(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list1.append(f)
        # 仅记录1000个数据点
        del self.state_f_list1[0]

    def force_callback2(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list2.append(f)
        # 仅记录1000个数据点
        del self.state_f_list2[0]

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
        if (self.robot1_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list1)
            plot_f = np.array(self.state_f_list1)
            self.plot_joint(plot_t, plot_qq)
            self.plot_force(plot_t, plot_f)

        if (self.robot2_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list2)
            plot_f = np.array(self.state_f_list2)
            self.plot_joint(plot_t, plot_qq)
            self.plot_force(plot_t, plot_f)

    def run_topic(self):
        # 读取时间
        self.T = float(self.lineEdit_T.text())
        self.t = float(self.lineEdit_t.text())

        msg_time = "获取时间：\n" + "周期T：" + str(self.T) + \
                   "\n规划时长：" + str(self.t) + "\n"

        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos1_path, JointState, self.joint_callback1)
        rospy.Subscriber(self.sub_force1_path, WrenchStamped, self.force_callback1)
        self.pub1 = rospy.Publisher(self.pub1_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos2_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force2_path, WrenchStamped, self.force_callback2)
        self.pub2 = rospy.Publisher(self.pub2_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"

        msg = msg_time + msg_tip
        self.textEdit.setText(msg)
        t1.start()

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        kk = 0
        command_qq_list1 = []
        command_qq_list2 = []

        # 求取数据长度
        if (self.robot1_flag):
            if(self.init_flag):
                kk = len(self.command_qq1_init)
                command_qq_list1 = self.command_qq1_init
            elif(self.home_flag):
                kk = len(self.command_qq1_home)
                command_qq_list1 = self.command_qq1_home
            else:
                kk = len(self.robot1_command_qq)
                command_qq_list1 = self.robot1_command_qq
        if (self.robot2_flag):
            if (self.init_flag):
                kk = len(self.command_qq2_init)
                command_qq_list2 = self.command_qq2_init
            elif (self.home_flag):
                kk = len(self.command_qq2_home)
                command_qq_list2 = self.command_qq2_home
            else:
                kk = len(self.robot2_command_qq)
                command_qq_list2 = self.robot2_command_qq
        if (self.all_flag):
            if (self.init_flag):
                kk = len(self.command_qq1_init)
                command_qq_list1 = self.command_qq1_init
                command_qq_list2 = self.command_qq2_init
            elif (self.home_flag):
                kk = len(self.command_qq1_home)
                command_qq_list1 = self.command_qq1_home
                command_qq_list2 = self.command_qq2_home
            else:
                kk = len(self.robot1_command_qq)
                command_qq_list1 = self.robot1_command_qq
                command_qq_list2 = self.robot2_command_qq
        # 进度条显示时间间隔
        show_time = int(kk * self.T * 10)

        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(show_time)

        # 设置绘图,用Qtimer开线程处理（线程4）
        self.timer_plot = QTimer()
        self.timer_plot.timeout.connect(self.realtime_plot)
        self.timer_plot.start(1000)

        # 发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                self.timer_p.stop()
                self.timer_plot.stop()
                break
            # 发送数据
            msg = "已发送机器人：\n"
            if (self.robot1_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = command_qq_list1[k, 0:self.n]
                else:
                    command_data.data = command_qq_list1[-1, 0:self.n]
                self.pub1.publish(command_data)
                if (k == 0):
                    msg = msg + "robot1已发送！\n"
            if (self.robot2_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = command_qq_list2[k, 0:self.n]
                else:
                    command_data.data = command_qq_list2[-1, 0:self.n]
                self.pub2.publish(command_data)
                if (k == 0):
                    msg = msg + "robot2已发送！\n"

            if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
                msg = "没有选中机械臂！\n"

            self.textEdit.setText(msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    # ===============窗口跳转函数================#
    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================三个UR5协同控制================#
class UrsWindow3(QMainWindow, Ui_UrsForm3):
    # 建立全局变量
    state_qq_list1 = list(np.zeros([1000, 6]))
    state_f_list1 = list(np.zeros([1000, 6]))
    state_qq_list2 = list(np.zeros([1000, 6]))
    state_f_list2 = list(np.zeros([1000, 6]))
    state_qq_list3 = list(np.zeros([1000, 6]))
    state_f_list3 = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0.0

    def __init__(self, parent=None):
        super(UrsWindow3, self).__init__(parent)
        self.T = 0.01
        self.run_flag = True  # 开始或停止标签
        self.real_flag = False
        self.gazebo_flag = False

        self.robot1_flag = False
        self.robot2_flag = False
        self.robot3_flag = False
        self.all_flag = False

        self.robot1_2_flag = False
        self.robot2_2_flag = False
        self.robot3_2_flag = False

        self.sub_force1_path = "/robot1/ft_sensor_topic"
        self.sub_pos1_path = "/robot1/joint_states"
        self.pub1_path = "/robot1/ur5_position_controller/command"
        self.sub_force2_path = "/robot2/ft_sensor_topic"
        self.sub_pos2_path = "/robot2/joint_states"
        self.pub2_path = "/robot2/ur5_position_controller/command"
        self.sub_force3_path = "/robot3/ft_sensor_topic"
        self.sub_pos3_path = "/robot3/joint_states"
        self.pub3_path = "/robot3/ur5_position_controller/command"

        self.n = 6  # 机械臂关节数
        self.qq_wish_pos = np.zeros([3, 6])
        self.qq_wish_state = np.zeros([3, 6])

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        mainMenu = menubar.addMenu('&Main')

        # 文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window ', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        mainMenu.addAction(openMain)

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_read.clicked.connect(self.read_wish_pos)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_refresh.clicked.connect(self.refresh_radioButton)
        self.button_plan.clicked.connect(self.plan)
        self.button_home.clicked.connect(self.home)
        self.button_init.clicked.connect(self.init)
        self.button_state.clicked.connect(self.get_current_state)

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

        # 绘制速度图
        self.p2.plot(t2, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t2, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t2, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t2, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t2, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t2, f[:, 5], pen='y', name='F6', clear=False)

    # 刷新按钮
    def refresh_radioButton(self):
        self.robot1_flag = self.radioButton_robot1.isChecked()
        self.robot2_flag = self.radioButton_robot2.isChecked()
        self.robot3_flag = self.radioButton_robot3.isChecked()
        self.all_flag = self.radioButton_all.isChecked()

        self.robot1_2_flag = self.radioButton_robot1_2.isChecked()
        self.robot2_2_flag = self.radioButton_robot2_2.isChecked()
        self.robot3_2_flag = self.radioButton_robot3_2.isChecked()

        self.real_flag = self.radioButton_real.isChecked()
        self.gazebo_flag = self.radioButton_gazebo.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

    # 读取给定关节角度
    def read_wish_pos(self):
        qq = np.zeros(6)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()

        qq_state = np.zeros(6)
        qq_state[0] = self.lineEdit_q1_3.text()
        qq_state[1] = self.lineEdit_q2_3.text()
        qq_state[2] = self.lineEdit_q3_3.text()
        qq_state[3] = self.lineEdit_q4_3.text()
        qq_state[4] = self.lineEdit_q5_3.text()
        qq_state[5] = self.lineEdit_q6_3.text()

        msg = ""
        if (self.robot1_flag or self.all_flag):
            msg_pos1 = "robot1规划目标点:\n" + \
                       "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                       "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                       "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                       "\n"

            self.qq_wish_pos[0, :] = np.copy(qq * np.pi / 180.0)
            self.qq_wish_state[0, :] = np.copy(qq_state * np.pi / 180.0)
            msg = msg + msg_pos1

        if (self.robot2_flag or self.all_flag):
            msg_pos = "robot2规划目标点:\n" + \
                      "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                      "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                      "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                      "\n"
            self.qq_wish_pos[1, :] = np.copy(qq * np.pi / 180.0)
            self.qq_wish_state[1, :] = np.copy(qq_state * np.pi / 180.0)
            msg = msg + msg_pos

        if (self.robot3_flag or self.all_flag):
            msg_pos = "robot3规划目标点:\n" + \
                      "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                      "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                      "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                      "\n"
            self.qq_wish_pos[2, :] = np.copy(qq * np.pi / 180.0)
            self.qq_wish_state[2, :] = np.copy(qq_state * np.pi / 180.0)
            msg = msg + msg_pos

        if (not (self.robot1_flag or self.robot2_flag or self.robot3_flag or self.all_flag)):
            msg = "没有选中机械臂！！！"

        self.textEdit.setText(msg)

    #获取当前状态
    def get_current_state(self):
        qq = np.zeros(6)
        if(self.robot1_flag or self.all_flag):
            qq = np.array(self.state_qq_list1[-1])

        if (self.robot2_flag or self.all_flag):
            qq = np.array(self.state_qq_list2[-1])

        if (self.robot3_flag or self.all_flag):
            qq = np.array(self.state_qq_list3[-1])

        qq = np.around(qq*180/np.pi, 3)
        #采集关节角存在问题，1、3互换
        self.lineEdit_q1_3.setText(str(qq[2]))
        self.lineEdit_q2_3.setText(str(qq[1]))
        self.lineEdit_q3_3.setText(str(qq[0]))
        self.lineEdit_q4_3.setText(str(qq[3]))
        self.lineEdit_q5_3.setText(str(qq[4]))
        self.lineEdit_q6_3.setText(str(qq[5]))

    # 给定初始位置
    def init(self):
        msg = ""
        if (self.robot1_flag or self.all_flag):
            qq = np.array([-90, -120, -120, 60, 90, 0])
            self.lineEdit_q1.setText(str(qq[0]))
            self.lineEdit_q2.setText(str(qq[1]))
            self.lineEdit_q3.setText(str(qq[2]))
            self.lineEdit_q4.setText(str(qq[3]))
            self.lineEdit_q5.setText(str(qq[4]))
            self.lineEdit_q6.setText(str(qq[5]))

        if (self.robot2_flag or self.all_flag):
            qq = np.array([-90, -60, 120, 120, -90, 0])
            self.lineEdit_q1.setText(str(qq[0]))
            self.lineEdit_q2.setText(str(qq[1]))
            self.lineEdit_q3.setText(str(qq[2]))
            self.lineEdit_q4.setText(str(qq[3]))
            self.lineEdit_q5.setText(str(qq[4]))
            self.lineEdit_q6.setText(str(qq[5]))

        if (self.robot3_flag or self.all_flag):
            qq = np.array([0, -90, -90, -90, 90, 0])
            self.lineEdit_q1.setText(str(qq[0]))
            self.lineEdit_q2.setText(str(qq[1]))
            self.lineEdit_q3.setText(str(qq[2]))
            self.lineEdit_q4.setText(str(qq[3]))
            self.lineEdit_q5.setText(str(qq[4]))
            self.lineEdit_q6.setText(str(qq[5]))

        if (not (self.robot1_flag or self.robot2_flag or self.robot3_flag or self.all_flag)):
            msg = "没有选中机械臂！\n"

    # 给定家点位置
    def home(self):
        qq = np.array([0, -90, 0, -90, 0, 0])
        self.lineEdit_q1.setText(str(qq[0]))
        self.lineEdit_q2.setText(str(qq[1]))
        self.lineEdit_q3.setText(str(qq[2]))
        self.lineEdit_q4.setText(str(qq[3]))
        self.lineEdit_q5.setText(str(qq[4]))
        self.lineEdit_q6.setText(str(qq[5]))

    # 规划函数
    def plan(self):
        msg = ""
        if (self.robot1_flag or self.all_flag):
            # 调用规划函数
            [qq, qv, qa] = gf.q_joint_space_plan_time(self.qq_wish_state[0, :], self.qq_wish_pos[0, :])
            # 调用绘图函数
            k = len(qq[:, 0])
            t = np.linspace(0, self.T * (k - 1), k)
            # 绘制关节角位置速度图
            self.plot_joint(t, qq, t, np.zeros([k, 6]))
            # 将规划好的位置定义为全局变量
            self.command_qq_list1 = np.copy(qq)
            msg1 = "robot1已规划！\n"
            msg = msg + msg1

        if (self.robot2_flag or self.all_flag):
            # 调用规划函数
            [qq, qv, qa] = gf.q_joint_space_plan_time(self.qq_wish_state[1, :], self.qq_wish_pos[1, :])
            # 调用绘图函数
            k = len(qq[:, 0])
            t = np.linspace(0, self.T * (k - 1), k)
            # 绘制关节角位置速度图
            self.plot_joint(t, qq, t, np.zeros([k, 6]))
            # 将规划好的位置定义为全局变量
            self.command_qq_list2 = np.copy(qq)
            msg2 = "robot2已规划！\n"
            msg = msg + msg2

        if (self.robot3_flag or self.all_flag):
            # 调用规划函数
            [qq, qv, qa] = gf.q_joint_space_plan_time(self.qq_wish_state[2, :], self.qq_wish_pos[2, :])
            # 调用绘图函数
            k = len(qq[:, 0])
            t = np.linspace(0, self.T * (k - 1), k)
            # 绘制关节角位置速度图
            self.plot_joint(t, qq, t, np.zeros([k, 6]))
            # 将规划好的位置定义为全局变量
            self.command_qq_list3 = np.copy(qq)
            msg3 = "robot3已规划！\n"
            msg = msg + msg3

        if (not (self.robot1_flag or self.robot2_flag or self.robot3_flag or self.all_flag)):
            msg = "没有选中机械臂！\n"

        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback1(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        self.state_t = self.state_t + self.T
        self.state_qq_list1.append(qq)
        self.state_qq1 = qq
        self.state_t_list.append(self.state_t)
        # 仅记录100个数据点
        del self.state_t_list[0]
        del self.state_qq_list1[0]

    def joint_callback2(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        self.state_qq_list2.append(qq)
        self.state_qq2 = qq
        # 仅记录100个数据点
        del self.state_qq_list2[0]

    def joint_callback3(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        self.state_qq_list3.append(qq)
        self.state_qq3 = qq
        # 仅记录100个数据点
        del self.state_qq_list3[0]

    ##关节角订阅回调函数
    def force_callback1(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list1.append(f)
        # 仅记录1000个数据点
        del self.state_f_list1[0]

    def force_callback2(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list2.append(f)
        # 仅记录1000个数据点
        del self.state_f_list2[0]

    def force_callback3(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list3.append(f)
        # 仅记录1000个数据点
        del self.state_f_list3[0]

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
        if (self.robot1_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list1)
            plot_f = np.array(self.state_f_list1)
            self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

        if (self.robot2_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list2)
            plot_f = np.array(self.state_f_list2)
            self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

        if (self.robot3_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list3)
            plot_f = np.array(self.state_f_list3)
            self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

    def run_topic(self):
        # 读取时间
        self.T = float(self.lineEdit_T.text())
        self.t = float(self.lineEdit_t.text())

        msg_time = "获取时间：\n" + "周期T：" + str(self.T) + \
                   "\n规划时长：" + str(self.t) + "\n"

        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos1_path, JointState, self.joint_callback1)
        rospy.Subscriber(self.sub_force1_path, WrenchStamped, self.force_callback1)
        self.pub1 = rospy.Publisher(self.pub1_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos2_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force2_path, WrenchStamped, self.force_callback2)
        self.pub2 = rospy.Publisher(self.pub2_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos3_path, JointState, self.joint_callback3)
        rospy.Subscriber(self.sub_force3_path, WrenchStamped, self.force_callback3)
        self.pub3 = rospy.Publisher(self.pub3_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"

        msg = msg_time + msg_tip
        self.textEdit.setText(msg)
        t1.start()

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        # 求取数据长度
        if (self.robot1_flag):
            kk = len(self.command_qq_list1)
        if (self.robot2_flag):
            kk = len(self.command_qq_list2)
        if (self.robot3_flag):
            kk = len(self.command_qq_list3)
        if (self.all_flag):
            kk = len(self.command_qq_list1)
        # 进度条显示时间间隔
        show_time = int(kk * self.T * 10)

        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(show_time)

        # 设置绘图,用Qtimer开线程处理（线程4）
        self.timer_plot = QTimer()
        self.timer_plot.timeout.connect(self.realtime_plot)
        self.timer_plot.start(1000)

        # 发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                self.timer_p.stop()
                self.timer_plot.stop()
                break
            # 发送数据
            msg = "已发送机器人：\n"
            if (self.robot1_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = self.command_qq_list1[k, 0:self.n]
                else:
                    command_data.data = self.command_qq_list1[-1, 0:self.n]
                self.pub1.publish(command_data)
                if (k == 0):
                    msg = msg + "robot1已发送！\n"
            if (self.robot2_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = self.command_qq_list2[k, 0:self.n]
                else:
                    command_data.data = self.command_qq_list2[-1, 0:self.n]
                self.pub2.publish(command_data)
                if (k == 0):
                    msg = msg + "robot2已发送！\n"
            if (self.robot3_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = self.command_qq_list3[k, 0:self.n]
                else:
                    command_data.data = self.command_qq_list3[-1, 0:self.n]
                self.pub3.publish(command_data)
                if (k == 0):
                    msg = msg + "robot3已发送！\n"

            if (not (self.robot1_flag or self.robot2_flag or self.robot3_flag or self.all_flag)):
                msg = "没有选中机械臂！\n"

            self.textEdit.setText(msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    # ===============窗口跳转函数================#
    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================两个自制机械臂的协同控制：关节空间控制================#
class ArmctWindow1(QMainWindow, Ui_ArmctForm1):
    # 建立全局变量
    state_qq_list1 = list(np.zeros([1000, 7]))
    state_f_list1 = list(np.zeros([1000, 6]))
    state_qq_list2 = list(np.zeros([1000, 7]))
    state_f_list2 = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0.0

    def __init__(self, parent=None):
        super(ArmctWindow1, self).__init__(parent)
        self.T = 0.01
        self.run_flag = True  # 开始或停止标签
        self.real_flag = False
        self.gazebo_flag = False

        self.robot1_flag = False
        self.robot2_flag = False
        self.all_flag = False

        self.force_flag = False
        self.pos_flag = False

        self.sub_force1_path = "/robot1/ft_sensor_topic"
        self.sub_pos1_path = "/robot1/joint_states"
        self.pub1_path = "/robot1/armt_position_controller/command"
        self.sub_force2_path = "/robot2/ft_sensor_topic"
        self.sub_pos2_path = "/robot2/joint_states"
        self.pub2_path = "/robot2/armc_position_controller/command"

        self.n = 7  # 机械臂关节数
        self.qq_wish_pos1 = np.zeros(7)
        self.qq_wish_pos2 = np.zeros(7)

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        mainMenu = menubar.addMenu('&Main')

        # 文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window ', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        mainMenu.addAction(openMain)

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_read.clicked.connect(self.read_wish_pos)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_refresh.clicked.connect(self.refresh_radioButton)
        self.button_plan.clicked.connect(self.plan)
        self.button_home.clicked.connect(self.home)
        self.button_init.clicked.connect(self.init)
        self.checkBox.stateChanged.connect(self.gazebo_or_real)

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
    def plot_joint1(self, t1, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='k', name='qq7', clear=False)

    def plot_joint2(self, t, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p2.plot(t, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p2.plot(t, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p2.plot(t, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p2.plot(t, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p2.plot(t, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p2.plot(t, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p2.plot(t, qq[:, 6], pen='k', name='qq7', clear=False)

    def plot_force1(self, t, f):
        # 绘制速度图
        self.p1.plot(t, f[:, 0], pen='b', name='F1', clear=True)
        self.p1.plot(t, f[:, 1], pen='g', name='F2', clear=False)
        self.p1.plot(t, f[:, 2], pen='r', name='F3', clear=False)
        self.p1.plot(t, f[:, 3], pen='c', name='F4', clear=False)
        self.p1.plot(t, f[:, 4], pen='m', name='F5', clear=False)
        self.p1.plot(t, f[:, 5], pen='y', name='F6', clear=False)

    def plot_force2(self, t, f):
        # 绘制速度图
        self.p2.plot(t, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t, f[:, 5], pen='y', name='F6', clear=False)

    def gazebo_or_real(self):
        self.real_flag = self.checkBox.isChecked()
        if(self.real_flag):
            self.sub_force1_path = "/armt/ft_sensor_topic"
            self.sub_pos1_path = "/armt/joint_states"
            self.pub1_path = "/armt/joint_command"
            self.sub_force2_path = "armt/ft_sensor_topic"
            self.sub_pos2_path = "/joint_states"
            self.pub2_path = "/all_joints_position_group_controller/command"
            msg = "选择实物"
            self.textEdit.setText(msg)
        else:
            self.sub_force1_path = "/robot1/ft_sensor_topic"
            self.sub_pos1_path = "/robot1/joint_states"
            self.pub1_path = "/robot1/armt_position_controller/command"
            self.sub_force2_path = "/robot2/ft_sensor_topic"
            self.sub_pos2_path = "/robot2/joint_states"
            self.pub2_path = "/robot2/armc_position_controller/command"
            msg = "选择仿真"
            self.textEdit.setText(msg)

    # 刷新按钮
    def refresh_radioButton(self):
        self.robot1_flag = self.radioButton_robot1.isChecked()
        self.robot2_flag = self.radioButton_robot2.isChecked()
        self.all_flag = self.radioButton_all.isChecked()

        self.force_flag = self.radioButton_force.isChecked()
        self.pos_flag = self.radioButton_pos.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

    # 读取给定关节角度
    def read_wish_pos(self):
        qq_t = np.zeros(7)
        qq_t[0] = self.lineEdit_q1_1.text()
        qq_t[1] = self.lineEdit_q2_1.text()
        qq_t[2] = self.lineEdit_q3_1.text()
        qq_t[3] = self.lineEdit_q4_1.text()
        qq_t[4] = self.lineEdit_q5_1.text()
        qq_t[5] = self.lineEdit_q6_1.text()
        qq_t[6] = self.lineEdit_q7_1.text()

        qq_c = np.zeros(7)
        qq_c[0] = self.lineEdit_q1_2.text()
        qq_c[1] = self.lineEdit_q2_2.text()
        qq_c[2] = self.lineEdit_q3_2.text()
        qq_c[3] = self.lineEdit_q4_2.text()
        qq_c[4] = self.lineEdit_q5_2.text()
        qq_c[5] = self.lineEdit_q6_2.text()
        qq_c[6] = self.lineEdit_q7_2.text()

        msg = ""
        if (self.robot1_flag or self.all_flag):
            msg_pos1 = "robot1规划目标点:\n" + \
                       "q1:" + str(qq_t[0]) + "\n" + "q2:" + str(qq_t[1]) + \
                       "\n" + "q3:" + str(qq_t[2]) + "\n" + "q4:" + str(qq_t[3]) + \
                       "\n" + "q5:" + str(qq_t[4]) + "\n" + "q6:" + str(qq_t[5]) + \
                       "\n" + "q7:" + str(qq_t[6]) + "\n"
            self.qq_wish_pos1 = np.copy(qq_t * np.pi / 180.0)
            msg = msg + msg_pos1

        if (self.robot2_flag or self.all_flag):
            msg_pos2 = "robot2规划目标点:\n" + \
                      "q1:" + str(qq_c[0]) + "\n" + "q2:" + str(qq_c[1]) + \
                      "\n" + "q3:" + str(qq_c[2]) + "\n" + "q4:" + str(qq_c[3]) + \
                      "\n" + "q5:" + str(qq_c[4]) + "\n" + "q6:" + str(qq_c[5]) + \
                      "\n" + "q7:" + str(qq_c[6]) + "\n"
            self.qq_wish_pos2 = np.copy(qq_c * np.pi / 180.0)
            msg = msg + msg_pos2


        if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
            msg = "没有选中机械臂！！！"

        self.textEdit.setText(msg)

    # 给定初始位置
    def init(self):
        qq = np.array([0, -30, 0, 90, 0, 30, 0])
        self.lineEdit_q1_1.setText(str(qq[0]))
        self.lineEdit_q2_1.setText(str(qq[1]))
        self.lineEdit_q3_1.setText(str(qq[2]))
        self.lineEdit_q4_1.setText(str(qq[3]))
        self.lineEdit_q5_1.setText(str(qq[4]))
        self.lineEdit_q6_1.setText(str(qq[5]))
        self.lineEdit_q7_1.setText(str(qq[6]))

        qq = np.array([0, -30, 0, 90, 0, 30, 0])
        self.lineEdit_q1_2.setText(str(qq[0]))
        self.lineEdit_q2_2.setText(str(qq[1]))
        self.lineEdit_q3_2.setText(str(qq[2]))
        self.lineEdit_q4_2.setText(str(qq[3]))
        self.lineEdit_q5_2.setText(str(qq[4]))
        self.lineEdit_q6_2.setText(str(qq[5]))
        self.lineEdit_q7_2.setText(str(qq[6]))

    # 给定家点位置
    def home(self):
        qq = np.array([0, 0, 0, 0, 0, 0, 0])
        self.lineEdit_q1_1.setText(str(qq[0]))
        self.lineEdit_q2_1.setText(str(qq[1]))
        self.lineEdit_q3_1.setText(str(qq[2]))
        self.lineEdit_q4_1.setText(str(qq[3]))
        self.lineEdit_q5_1.setText(str(qq[4]))
        self.lineEdit_q6_1.setText(str(qq[5]))
        self.lineEdit_q7_1.setText(str(qq[6]))

        qq = np.array([0, 0, 0, 0, 0, 0, 0])
        self.lineEdit_q1_2.setText(str(qq[0]))
        self.lineEdit_q2_2.setText(str(qq[1]))
        self.lineEdit_q3_2.setText(str(qq[2]))
        self.lineEdit_q4_2.setText(str(qq[3]))
        self.lineEdit_q5_2.setText(str(qq[4]))
        self.lineEdit_q6_2.setText(str(qq[5]))
        self.lineEdit_q7_2.setText(str(qq[6]))
    # 规划函数
    def plan(self):
        msg = ""
        if (self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq, qv, qa] = gf.q_joint_space_plan_time(qq_b, self.qq_wish_pos1)
            # 调用绘图函数
            k = len(qq[:, 0])
            t = np.linspace(0, self.T * (k - 1), k)
            # 绘制关节角位置速度图
            self.plot_joint1(t, qq)
            # 将规划好的位置定义为全局变量
            self.command_qq_list1 = np.copy(qq)
            msg1 = "robot1已规划！\n"
            msg = msg + msg1

        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq, qv, qa] = gf.q_joint_space_plan_time(qq_b, self.qq_wish_pos2)
            # 调用绘图函数
            k = len(qq[:, 0])
            t = np.linspace(0, self.T * (k - 1), k)
            # 绘制关节角位置速度图
            self.plot_joint2(t, qq)
            # 将规划好的位置定义为全局变量
            self.command_qq_list2 = np.copy(qq)
            msg2 = "robot2已规划！\n"
            msg = msg + msg2

        if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
            msg = "没有选中机械臂！\n"

        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback1(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        self.state_t = self.state_t + self.T
        self.state_qq_list1.append(qq)
        self.state_t_list.append(self.state_t)
        # 仅记录100个数据点
        del self.state_t_list[0]
        del self.state_qq_list1[0]

    def joint_callback2(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]
        self.state_qq_list2.append(qq)
        # 仅记录100个数据点
        del self.state_qq_list2[0]

    ##关节角订阅回调函数
    def force_callback1(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list1.append(f)
        # 仅记录1000个数据点
        del self.state_f_list1[0]

    def force_callback2(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list2.append(f)
        # 仅记录1000个数据点
        del self.state_f_list2[0]

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
        if (self.force_flag):
            plot_t = np.array(self.state_t_list)
            plot_f1 = np.array(self.state_f_list1)
            plot_f2 = np.array(self.state_f_list2)
            self.plot_force1(plot_t, plot_f1)
            self.plot_force2(plot_t, plot_f2)

        if (self.pos_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq1 = np.array(self.state_qq_list1)
            plot_qq2 = np.array(self.state_qq_list2)
            self.plot_joint1(plot_t, plot_qq1)
            self.plot_joint2(plot_t, plot_qq2)

    def run_topic(self):
        # 读取时间
        self.T = float(self.lineEdit_T.text())
        self.t = float(self.lineEdit_t.text())

        msg_time = "获取时间：\n" + "周期T：" + str(self.T) + \
                   "\n规划时长：" + str(self.t) + "\n"

        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos1_path, JointState, self.joint_callback1)
        rospy.Subscriber(self.sub_force1_path, WrenchStamped, self.force_callback1)
        self.pub1 = rospy.Publisher(self.pub1_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos2_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force2_path, WrenchStamped, self.force_callback2)
        self.pub2 = rospy.Publisher(self.pub2_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"

        msg = msg_time + msg_tip
        self.textEdit.setText(msg)
        t1.start()

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True
        kk = 0
        # 求取数据长度
        if (self.robot1_flag):
            kk = len(self.command_qq_list1)
        if (self.robot2_flag):
            kk = len(self.command_qq_list2)
        if (self.all_flag):
            kk = len(self.command_qq_list1)

        if(not self.real_flag):
            # 进度条显示时间间隔
            show_time = int(kk * self.T * 10)

            # 设置ProgressBar,用Qtimer开线程处理（线程3）
            self.step_p = 0
            self.timer_p = QTimer()
            self.timer_p.timeout.connect(self.probar_show)
            self.timer_p.start(show_time)

            # 设置绘图,用Qtimer开线程处理（线程4）
            self.timer_plot = QTimer()
            self.timer_plot.timeout.connect(self.realtime_plot)
            self.timer_plot.start(1000)

        # 发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                if(not self.real_flag):
                    self.timer_p.stop()
                    self.timer_plot.stop()
                break
            # 发送数据
            msg = "已发送机器人：\n"
            if (self.robot1_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = self.command_qq_list1[k, 0:self.n]
                else:
                    break
                self.pub1.publish(command_data)
                if (k == 0):
                    msg = msg + "robot1已发送！\n"
            if (self.robot2_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = self.command_qq_list2[k, 0:self.n]
                else:
                    break
                self.pub2.publish(command_data)
                if (k == 0):
                    msg = msg + "robot2已发送！\n"

            if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
                msg = "没有选中机械臂！\n"
                break

            self.textEdit.setText(msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

        msg = "运动完成！\n"
        self.textEdit.setText(msg)

    # ===============窗口跳转函数================#
    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================两个自制机械臂的协同控制：笛卡尔空间控制================#
class ArmctWindow2(QMainWindow, Ui_ArmctForm2):
    # 建立全局变量
    state_qq_list1 = list(np.zeros([1000, 7]))
    state_f_list1 = list(np.zeros([1000, 6]))
    state_qq_list2 = list(np.zeros([1000, 7]))
    state_f_list2 = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0.0

    def __init__(self, parent=None):
        super(ArmctWindow2, self).__init__(parent)
        self.T = 0.01
        self.t = 10
        self.run_flag = False  # 开始或停止标签
        self.real_flag = False

        self.robot1_flag = False
        self.robot2_flag = False
        self.all_flag = False

        self.robot1_2_flag = False
        self.robot2_2_flag = False

        self.read_pos_flag = False

        self.init_flag = False
        self.data_flag = False
        self.home_flag = False

        self.sub_force1_path = "/robot1/ft_sensor_topic"
        self.sub_pos1_path = "/robot1/joint_states"
        self.pub1_path = "/robot1/armt_position_controller/command"
        self.sub_force2_path = "/robot2/ft_sensor_topic"
        self.sub_pos2_path = "/robot2/joint_states"
        self.pub2_path = "/robot2/armc_position_controller/command"

        self.n = 7  # 机械臂关节数

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        mainMenu = menubar.addMenu('&Main')

        # 文件菜单:返回主窗口
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
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_read.clicked.connect(self.read_data)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_refresh.clicked.connect(self.refresh_radioButton)
        self.button_plan_init.clicked.connect(self.go_init)
        self.button_plan_data.clicked.connect(self.run_data)
        self.button_plan_home.clicked.connect(self.go_home)
        self.checkBox.stateChanged.connect(self.gazebo_or_real)

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

    # 打开文件的地址和内容
    def fileOpen(self):
        # 打开文件操作
        path = os.path.join(os.getcwd(), '../', 'data/robots/armct')
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

    # 绘画关节角和关节角速度曲线
    def plot_joint1(self, t1, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='k', name='qq7', clear=False)

    def plot_joint2(self, t, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p2.plot(t, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p2.plot(t, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p2.plot(t, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p2.plot(t, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p2.plot(t, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p2.plot(t, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p2.plot(t, qq[:, 6], pen='k', name='qq7', clear=False)

    def plot_force(self, t, f):
        # 绘制速度图
        self.p2.plot(t, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t, f[:, 5], pen='y', name='F6', clear=False)

    def gazebo_or_real(self):
        self.real_flag = self.checkBox.isChecked()
        if (self.real_flag):
            self.sub_force1_path = "/armt/ft_sensor_topic"
            self.sub_pos1_path = "/armt/joint_states"
            self.pub1_path = "/armt/joint_command"
            self.sub_force2_path = "/armc/ft_sensor_topic"
            self.sub_pos2_path = "/joint_states"
            self.pub2_path = "/all_joints_position_group_controller/command"
            msg = "选择实物"
            self.textEdit.setText(msg)
        else:
            self.sub_force1_path = "/robot1/ft_sensor_topic"
            self.sub_pos1_path = "/robot1/joint_states"
            self.pub1_path = "/robot1/armt_position_controller/command"
            self.sub_force2_path = "/robot2/ft_sensor_topic"
            self.sub_pos2_path = "/robot2/joint_states"
            self.pub2_path = "/robot2/armc_position_controller/command"
            msg = "选择仿真"
            self.textEdit.setText(msg)

    # 刷新按钮
    def refresh_radioButton(self):
        self.robot1_flag = self.radioButton_robot1.isChecked()
        self.robot2_flag = self.radioButton_robot2.isChecked()
        self.all_flag = self.radioButton_all.isChecked()

        self.robot1_2_flag = self.radioButton_robot1_2.isChecked()
        self.robot2_2_flag = self.radioButton_robot2_2.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

    # 读取给定关节角度
    def read_data(self):
        # 读取数据
        armt_pos_path = str(self.lineEdit_qq_armt.text())
        armc_pos_path = str(self.lineEdit_qq_armc.text())

        self.command_qq1_data = fo.read(armt_pos_path)
        self.command_qq2_data = fo.read(armc_pos_path)

        # 调用绘图函数
        k = len(self.command_qq1_data)
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_joint1(t, self.command_qq1_data)
        self.plot_joint2(t, self.command_qq2_data)

        msg = "位置已读取！"
        self.read_pos_flag = True
        self.textEdit.setText(msg)

        self.textEdit.setText(msg)

    #运动到初始位置
    def go_init(self):
        self.run_flag = False

        self.init_flag = True
        self.data_flag = False
        self.home_flag = False
        if(not self.read_pos_flag):
            msg = "请先读取规划数据！"
            self.textEdit.setText(msg)
            return -1

        msg = ''
        if(self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq1, _, _] = gf.q_joint_space_plan_time(qq_b1, self.command_qq1_data[0, :],
                                                      self.T, self.t)
            # 调用绘图函数
            k1 = len(qq1[:, 0])
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            self.plot_joint1(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_init = np.copy(qq1)
            msg = msg + "armt运动到初始点已规划！\n"
        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2, _, _] = gf.q_joint_space_plan_time(qq_b2, self.command_qq2_data[0, :],
                                                     self.T, self.t)
            # 调用绘图函数
            k2 = len(qq2[:, 0])
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            self.plot_joint2(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_init = np.copy(qq2)
            msg = msg + "armc运动到初始点已规划！\n"
        if(not (self.robot1_flag or self.robot2_flag or self.all_flag)):
            msg = '没有选择机械臂，请选择机械臂并刷新按钮！'
        self.textEdit.setText(msg)

    def run_data(self):
        self.run_flag = False

        self.init_flag = False
        self.data_flag = True
        self.home_flag = False

        msg = "已切换到数据轨迹段！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        self.run_flag = False

        self.init_flag = False
        self.imp_flag = False
        self.home_flag = True

        # 获得规划起点
        qq_home = np.array([0, 0, 0, 0, 0, 0, 0.0])

        msg = ''
        if (self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq1, _, _] = gf.q_joint_space_plan_time(qq_b1, qq_home, self.T, self.t)
            # 调用绘图函数
            k1 = len(qq1[:, 0])
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            self.plot_joint1(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_home = np.copy(qq1)
            msg = msg + "armt运动到home点已规划！\n"

        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2, _, _] = gf.q_joint_space_plan_time(qq_b2, qq_home, self.T, self.t)
            # 调用绘图函数
            k2 = len(qq2[:, 0])
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            self.plot_joint2(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_home = np.copy(qq2)
            msg = msg + "armc运动到home点已规划！\n"

        if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
            msg = '没有选择机械臂，请选择机械臂并刷新按钮！'
        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback1(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        self.state_t = self.state_t + self.T
        self.state_qq_list1.append(qq)
        self.state_t_list.append(self.state_t)
        # 仅记录100个数据点
        del self.state_t_list[0]
        del self.state_qq_list1[0]

    def joint_callback2(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]
        self.state_qq_list2.append(qq)
        # 仅记录100个数据点
        del self.state_qq_list2[0]

    ##关节角订阅回调函数
    def force_callback1(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list1.append(f)
        # 仅记录1000个数据点
        del self.state_f_list1[0]

    def force_callback2(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list2.append(f)
        # 仅记录1000个数据点
        del self.state_f_list2[0]

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
        if (self.robot1_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list1)
            plot_f = np.array(self.state_f_list1)
            self.plot_joint1(plot_t, plot_qq)
            self.plot_force(plot_t, plot_f)

        if (self.robot2_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list2)
            plot_f = np.array(self.state_f_list2)
            self.plot_joint1(plot_t, plot_qq)
            self.plot_force(plot_t, plot_f)

    def run_topic(self):
        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos1_path, JointState, self.joint_callback1)
        rospy.Subscriber(self.sub_force1_path, WrenchStamped, self.force_callback1)
        self.pub1 = rospy.Publisher(self.pub1_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos2_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force2_path, WrenchStamped, self.force_callback2)
        self.pub2 = rospy.Publisher(self.pub2_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"

        msg = msg_tip
        self.textEdit.setText(msg)
        t1.start()

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        # 提示标语
        msg = "开始下发命令！\n"
        self.textEdit.setText(msg)

        if(not self.real_flag):
            # 设置绘图,用Qtimer开线程处理（线程4）
            self.timer_plot = QTimer()
            self.timer_plot.timeout.connect(self.realtime_plot)
            self.timer_plot.start(1000)

        # 发送关节角度
        if(self.robot1_flag or self.all_flag):
            if (self.init_flag):
                command_qq1 = np.copy(self.command_qq1_init)
            if (self.data_flag):
                command_qq1 = np.copy(self.command_qq1_data)
            if (self.home_flag):
                command_qq1 = np.copy(self.command_qq1_home)
            kk = len(command_qq1)

        if (self.robot2_flag or self.all_flag):
            if (self.init_flag):
                command_qq2 = np.copy(self.command_qq2_init)
            if (self.data_flag):
                command_qq2 = np.copy(self.command_qq2_data)
            if (self.home_flag):
                command_qq2 = np.copy(self.command_qq2_home)
            kk = len(command_qq2)

        if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
            msg = '没有选择机械臂，请选择机械臂并刷新按钮！'
            self.textEdit.setText(msg)
            return -1


        k = 0
        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(10*self.T*kk)

        rate = rospy.Rate(100)
        msg = ''
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag or k==kk):
                if(not self.real_flag):
                    self.timer_plot.stop()
                    self.timer_p.stop()
                break

            if (self.robot1_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = command_qq1[k, :]
                else:
                    command_data.data = command_qq1[-1, :]
                self.pub1.publish(command_data)
                if (k == 0):
                    msg =msg + "armt开始运动!\n"

            if (self.robot2_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = command_qq2[k, :]
                else:
                    command_data.data = command_qq2[-1, :]
                self.pub2.publish(command_data)
                if (k == 0):
                    msg = msg + "armc开始运动!\n"

            if (k == 0):
                self.textEdit.setText(msg)
            k = k + 1
            QApplication.processEvents()

            rate.sleep()

    # ===============窗口跳转函数================#
    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================两个自制机械臂的协同控制：阻抗+位置搬运控制================#
class ArmctWindow3(QMainWindow, Ui_ArmctForm3):
    # 建立全局变量
    state_qq_list1 = list(np.zeros([1000, 7]))
    state_f_list1 = list(np.zeros([1000, 6]))
    state_qq_list2 = list(np.zeros([1000, 7]))
    state_f_list2 = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0.0

    def __init__(self, parent=None):
        super(ArmctWindow3, self).__init__(parent)
        self.T = 0.01
        self.t = 10
        self.run_flag = False  # 开始或停止标签
        self.real_flag = False

        self.robot1_flag = False
        self.robot2_flag = False
        self.all_flag = False

        self.force_flag = False

        self.read_pos_flag = False

        self.init_flag = False
        self.data_flag = False
        self.home_flag = False

        self.sub_force1_path = "/robot1/ft_sensor_topic"
        self.sub_pos1_path = "/robot1/joint_states"
        self.pub1_path = "/robot1/armt_position_controller/command"
        self.sub_force2_path = "/robot2/ft_sensor_topic"
        self.sub_pos2_path = "/robot2/joint_states"
        self.pub2_path = "/robot2/armc_position_controller/command"

        self.n = 7  # 机械臂关节数

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        mainMenu = menubar.addMenu('&Main')

        # 文件菜单:返回主窗口
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
        self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_read.clicked.connect(self.read_data)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_refresh.clicked.connect(self.refresh_radioButton)
        self.button_plan_init.clicked.connect(self.go_init)
        self.button_plan_data.clicked.connect(self.run_data)
        self.button_plan_home.clicked.connect(self.go_home)
        self.checkBox_real.stateChanged.connect(self.gazebo_or_real)
        self.checkBox_force.stateChanged.connect(self.force_or_pos)

    # ===============按钮功能模块相关函数================#
    # 采用pyqtgraph绘制曲线,添加画板
    def set_graph_ui(self):
        pg.setConfigOptions(antialias=True)  # pg全局变量设置函数，antialias=True开启曲线抗锯齿

        self.win1 = pg.GraphicsLayoutWidget()  # 创建pg layout，可实现数据界面布局自动管理
        self.win2 = pg.GraphicsLayoutWidget()

        # pg绘图窗口可以作为一个widget添加到GUI中的graph_layout，当然也可以添加到Qt其他所有的容器中
        self.horizontalLayout_1.addWidget(self.win1)
        self.horizontalLayout_2.addWidget(self.win2)
        self.force_or_pos()

    # 打开文件的地址和内容
    def fileOpen(self):
        # 打开文件操作
        path = os.path.join(os.getcwd(), '../', 'data/robots/armct')
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

    #选择位置或六维力,设置对应画板
    def force_or_pos(self):
        self.win1.clear()
        self.win2.clear()
        self.force_flag = self.checkBox_force.isChecked()
        if(self.force_flag ):
            p1 = self.win1.addPlot(title="armt_force")  # 添加第一个绘图窗口
            p1.setLabel('left', text='force/N', color='#ffffff')  # y轴设置函数
            p1.showGrid(x=True, y=True)  # 栅格设置函数
            p1.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
            p1.setLabel('bottom', text='time', units='s')  # x轴设置函数
            p1.addLegend(size=(50, 30))

            p2 = self.win2.addPlot(title="armc_force")  # 添加第一个绘图窗口
            p2.setLabel('left', text='force/N', color='#ffffff')  # y轴设置函数
            p2.showGrid(x=True, y=True)  # 栅格设置函数
            p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
            p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
            p2.addLegend(size=(50, 30))

            self.p1 = p1
            self.p2 = p2

            msg = "选择绘制六维力"
            self.textEdit.setText(msg)
        else:
            p1 = self.win1.addPlot(title="armt_joint pos")  # 添加第一个绘图窗口
            p1.setLabel('left', text='pos/rad', color='#ffffff')  # y轴设置函数
            p1.showGrid(x=True, y=True)  # 栅格设置函数
            p1.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
            p1.setLabel('bottom', text='time', units='s')  # x轴设置函数
            p1.addLegend(size=(50, 30))  # 可选择是否添加legend

            p2 = self.win2.addPlot(title="armc_joint pos")  # 添加第一个绘图窗口
            p2.setLabel('left', text='pos/rad', color='#ffffff')  # y轴设置函数
            p2.showGrid(x=True, y=True)  # 栅格设置函数
            p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
            p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
            p2.addLegend(size=(50, 30))  # 可选择是否添加legend

            self.p1 = p1
            self.p2 = p2

            msg = "选择绘制位置"
            self.textEdit.setText(msg)

    # 绘画关节角和关节角速度曲线
    def plot_joint1(self, t1, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='k', name='qq7', clear=False)

    def plot_joint2(self, t, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p2.plot(t, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p2.plot(t, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p2.plot(t, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p2.plot(t, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p2.plot(t, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p2.plot(t, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p2.plot(t, qq[:, 6], pen='k', name='qq7', clear=False)

    def plot_force1(self, t, f):
        # 绘制速度图
        self.p1.plot(t, f[:, 0], pen='b', name='F1', clear=True)
        self.p1.plot(t, f[:, 1], pen='g', name='F2', clear=False)
        self.p1.plot(t, f[:, 2], pen='r', name='F3', clear=False)
        self.p1.plot(t, f[:, 3], pen='c', name='F4', clear=False)
        self.p1.plot(t, f[:, 4], pen='m', name='F5', clear=False)
        self.p1.plot(t, f[:, 5], pen='y', name='F6', clear=False)

    def plot_force2(self, t, f):
        #自带绘画版
        p2 = self.win2.addPlot(title="force")  # 添加第一个绘图窗口
        p2.setLabel('left', text='force/N', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        # 绘制速度图
        self.p2.plot(t, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t, f[:, 5], pen='y', name='F6', clear=False)

    def gazebo_or_real(self):
        self.real_flag = self.checkBox_real.isChecked()
        if (self.real_flag):
            self.sub_force1_path = "/armt/ft_sensor_topic"
            self.sub_pos1_path = "/armt/joint_states"
            self.pub1_path = "/armt/joint_command"
            self.sub_force2_path = "/armc/ft_sensor_topic"
            self.sub_pos2_path = "/joint_states"
            self.pub2_path = "/all_joints_position_group_controller/command"
            msg = "选择实物"
            self.textEdit.setText(msg)
        else:
            self.sub_force1_path = "/robot1/ft_sensor_topic"
            self.sub_pos1_path = "/robot1/joint_states"
            self.pub1_path = "/robot1/armt_position_controller/command"
            self.sub_force2_path = "/robot2/ft_sensor_topic"
            self.sub_pos2_path = "/robot2/joint_states"
            self.pub2_path = "/robot2/armc_position_controller/command"
            msg = "选择仿真"
            self.textEdit.setText(msg)

    # 刷新按钮
    def refresh_radioButton(self):
        self.robot1_flag = self.radioButton_robot1.isChecked()
        self.robot2_flag = self.radioButton_robot2.isChecked()
        self.all_flag = self.radioButton_all.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

    # 读取给定关节角度
    def read_data(self):
        # 读取数据
        armt_pos_path = str(self.lineEdit_qq_armt.text())
        armc_pos_path = str(self.lineEdit_qq_armc.text())
        armt_f_path = str(self.lineEdit_f_armt.text())
        armc_f_path = str(self.lineEdit_f_armc.text())

        self.command_qq1_data = fo.read(armt_pos_path)
        self.command_qq2_data = fo.read(armc_pos_path)
        self.command_f1_data = fo.read(armt_f_path)
        self.command_f2_data = fo.read(armc_f_path)

        # 调用绘图函数
        self.plot_plan()

        msg = "位置已读取！"
        self.read_pos_flag = True
        self.textEdit.setText(msg)

        self.textEdit.setText(msg)

    #绘制规划图
    def plot_plan(self):
        self.force_flag = self.checkBox_force.isChecked()
        if(self.force_flag):
            # 调用绘图函数
            k1 = len(self.command_f1_data)
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            # 绘制关节角位置速度图
            self.plot_force1(t1, self.command_f1_data)
            self.plot_force2(t1, self.command_f2_data)
        else:
            # 调用绘图函数
            k2 = len(self.command_qq1_data)
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            # 绘制关节角位置速度图
            self.plot_joint1(t2, self.command_qq1_data)
            self.plot_joint2(t2, self.command_qq2_data)

    # 运动到初始位置
    def go_init(self):
        self.run_flag = False

        self.init_flag = True
        self.data_flag = False
        self.home_flag = False
        if (not self.read_pos_flag):
            msg = "请先读取规划数据！"
            self.textEdit.setText(msg)
            return -1

        msg = ''
        if (self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq1, _, _] = gf.q_joint_space_plan_time(qq_b1, self.command_qq1_data[0, :],
                                                     self.T, self.t)
            # 调用绘图函数
            k1 = len(qq1[:, 0])
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            self.plot_joint1(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_init = np.copy(qq1)
            msg = msg + "armt运动到初始点已规划！\n"
        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2, _, _] = gf.q_joint_space_plan_time(qq_b2, self.command_qq2_data[0, :],
                                                     self.T, self.t)
            # 调用绘图函数
            k2 = len(qq2[:, 0])
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            self.plot_joint2(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_init = np.copy(qq2)
            msg = msg + "armc运动到初始点已规划！\n"
        if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
            msg = '没有选择机械臂，请选择机械臂并刷新按钮！'
        self.textEdit.setText(msg)

    def run_data(self):
        self.run_flag = False

        self.init_flag = False
        self.data_flag = True
        self.home_flag = False

        msg = "已切换到阻抗数据轨迹段！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        self.run_flag = False

        self.init_flag = False
        self.data_flag = False
        self.home_flag = True

        # 获得规划起点
        qq_home = np.array([0, 0, 0, 0, 0, 0, 0.0])

        msg = ''
        if (self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq1, _, _] = gf.q_joint_space_plan_time(qq_b1, qq_home, self.T, self.t)
            # 调用绘图函数
            k1 = len(qq1[:, 0])
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            self.plot_joint1(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_home = np.copy(qq1)
            msg = msg + "armt运动到home点已规划！\n"

        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2, _, _] = gf.q_joint_space_plan_time(qq_b2, qq_home, self.T, self.t)
            # 调用绘图函数
            k2 = len(qq2[:, 0])
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            self.plot_joint2(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_home = np.copy(qq2)
            msg = msg + "armc运动到home点已规划！\n"
        if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
            msg = '没有选择机械臂，请选择机械臂并刷新按钮！'
        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback1(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        self.state_t = self.state_t + self.T
        self.state_qq_list1.append(qq)
        self.state_t_list.append(self.state_t)
        # 仅记录100个数据点
        del self.state_t_list[0]
        del self.state_qq_list1[0]

    def joint_callback2(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]
        qq[0] = 0.0
        self.state_qq_list2.append(qq)
        # 仅记录100个数据点
        del self.state_qq_list2[0]

    ##关节角订阅回调函数
    def force_callback1(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list1.append(f)
        # 仅记录1000个数据点
        del self.state_f_list1[0]

    def force_callback2(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list2.append(f)
        # 仅记录1000个数据点
        del self.state_f_list2[0]

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
        self.force_flag = self.checkBox_force.isChecked()
        if (self.force_flag):
            plot_t = np.array(self.state_t_list)
            plot_f1 = np.array(self.state_f_list1)
            plot_f2 = np.array(self.state_f_list2)
            self.plot_force1(plot_t, plot_f1)
            self.plot_force2(plot_t, plot_f2)
        else:
            plot_t = np.array(self.state_t_list)
            plot_qq1 = np.array(self.state_qq_list1)
            plot_qq2 = np.array(self.state_qq_list2)
            self.plot_joint1(plot_t, plot_qq1)
            self.plot_joint2(plot_t, plot_qq2)

    def run_topic(self):
        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos1_path, JointState, self.joint_callback1)
        rospy.Subscriber(self.sub_force1_path, WrenchStamped, self.force_callback1)
        self.pub1 = rospy.Publisher(self.pub1_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos2_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force2_path, WrenchStamped, self.force_callback2)
        self.pub2 = rospy.Publisher(self.pub2_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"

        msg = msg_tip
        self.textEdit.setText(msg)
        t1.start()

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        # 提示标语
        msg = "开始下发命令！\n"
        self.textEdit.setText(msg)

        # 创建阻抗自适应阻抗实例
        imp_arm1 = imp.IIMPController_iter_TCP()

        # 获取阻抗参数
        self.M = np.array([0.0, 0.0, 4.0, 0.0, 0.0, 0.0])
        self.B = np.array([0.0, 0.0, 800.0, 0.0, 0.0, 0.0])
        self.K = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.I = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # 输入参数
        robot = 'armc'
        [DH0, q_max, q_min] = gf.get_robot_parameter(robot)
        imp_arm1.get_robot_parameter(DH0, q_max, q_min)
        imp_arm1.get_period(self.T)
        # 实时调整阻抗参数
        imp_arm1.get_imp_parameter(self.M, self.B, self.K, self.I)

        if (not self.real_flag):
            # 设置绘图,用Qtimer开线程处理（线程4）
            self.timer_plot = QTimer()
            self.timer_plot.timeout.connect(self.realtime_plot)
            self.timer_plot.start(1000)

        # 发送关节角度
        command_qq1 = 0
        command_qq2 = 0

        if (self.robot1_flag or self.all_flag):
            if (self.init_flag):
                command_qq1 = np.copy(self.command_qq1_init)
            if (self.data_flag):
                command_qq1 = np.copy(self.command_qq1_data)
            if (self.home_flag):
                command_qq1 = np.copy(self.command_qq1_home)

        if (self.robot2_flag or self.all_flag):
            if (self.init_flag):
                command_qq2 = np.copy(self.command_qq2_init)
            if (self.data_flag):
                command_qq2 = np.copy(self.command_qq2_data)
            if (self.home_flag):
                command_qq2 = np.copy(self.command_qq2_home)

        if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
            msg = '没有选择机械臂，请选择机械臂并刷新按钮！'
            self.textEdit.setText(msg)
            return -1

        kk = len(command_qq1)
        k = 0
        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(10 * self.T * kk)

        rate = rospy.Rate(100)
        msg = ''
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                if (not self.real_flag):
                    self.timer_plot.stop()
                    self.timer_p.stop()
                break

            if (self.robot1_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = command_qq1[k, :]
                else:
                    break
                self.pub1.publish(command_data)
                if (k == 0):
                    msg = msg + "armt开始运动!\n"

            if (self.robot2_flag or self.all_flag):
                command_data = Float64MultiArray()
                if(self.data_flag):
                    if (k < kk):
                        qq = command_qq2[k, :]
                        fd = self.command_f2_data[k, :]
                    else:
                        qq = command_qq2[-1, :]
                        fd = self.command_f2_data[-1, :]
                    imp_arm1.get_expect_joint(qq)
                    imp_arm1.get_current_joint(np.array(self.state_qq_list2[-1]))
                    # 读取当前关节角和力
                    imp_arm1.get_expect_force(fd)
                    imp_arm1.get_current_force(np.array(self.state_f_list2[-1]))
                    # 计算修正关节角
                    qr = imp_arm1.compute_imp_joint()
                    # 发送数据

                    command_data.data = qr
                else:
                    if (k < kk):
                        command_data.data = command_qq2[k, :]
                    else:
                        break

                self.pub2.publish(command_data)
                if (k == 0):
                    msg = msg + "armc开始运动!\n"

            if (k == 0):
                self.textEdit.setText(msg)
            k = k + 1
            QApplication.processEvents()

            rate.sleep()
        msg = "运行完成！"
        self.textEdit.setText(msg)

    # ===============窗口跳转函数================#
    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================三个UR5协同打磨================#
class TechnologyWindow1(QMainWindow, Ui_TechnologyForm1):
    # 建立全局变量
    state_qq_list1 = list(np.zeros([1000, 6]))
    state_f_list1 = list(np.zeros([1000, 6]))
    state_qq_list2 = list(np.zeros([1000, 6]))
    state_f_list2 = list(np.zeros([1000, 6]))
    state_qq_list3 = list(np.zeros([1000, 6]))
    state_f_list3 = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0.0

    def __init__(self, parent=None):
        super(TechnologyWindow1, self).__init__(parent)
        self.T = 0.01
        self.run_flag = True  # 开始或停止标签
        self.real_flag = False
        self.gazebo_flag = False

        self.robot1_flag = False
        self.robot2_flag = False
        self.robot3_flag = False
        self.all_flag = True

        self.robot1_2_flag = True
        self.robot2_2_flag = False
        self.robot3_2q_flag = False
        self.robot3_2f_flag = False

        self.init_flag = False
        self.move_flag = False
        self.home_flag = False

        self.state_qq1 = np.zeros(6)
        self.state_qq2 = np.zeros(6)

        self.sub_force1_path = "/robot1/ft_sensor_topic"
        self.sub_pos1_path = "/robot1/joint_states"
        self.pub1_path = "/robot1/ur5_position_controller/command"
        self.sub_force2_path = "/robot2/ft_sensor_topic"
        self.sub_pos2_path = "/robot2/joint_states"
        self.pub2_path = "/robot2/ur5_position_controller/command"
        self.sub_force3_path = "/robot3/ft_sensor_topic"
        self.sub_pos3_path = "/robot3/joint_states"
        self.pub3_path = "/robot3/ur5_position_controller/command"

        self.n = 6  # 机械臂关节数

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        mainMenu = menubar.addMenu('&Main')

        # 文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window ', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        mainMenu.addAction(openMain)

        # =======================绘图相关设置=======================#
        self.p1 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_read_data.clicked.connect(self.read_pos)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_polish.clicked.connect(self.run_polish)
        self.button_refresh.clicked.connect(self.refresh_radioButton)
        self.button_work.clicked.connect(self.go_move)
        self.button_init_plan.clicked.connect(self.go_init)
        self.button_end_plan.clicked.connect(self.go_home)
        self.button_data_plot.clicked.connect(self.plot_data)

    # ===============按钮功能模块相关函数================#
    # 采用pyqtgraph绘制曲线,添加画板
    def set_graph_ui(self):
        pg.setConfigOptions(antialias=True)  # pg全局变量设置函数，antialias=True开启曲线抗锯齿

        win1 = pg.GraphicsLayoutWidget()  # 创建pg layout，可实现数据界面布局自动管理

        # pg绘图窗口可以作为一个widget添加到GUI中的graph_layout，当然也可以添加到Qt其他所有的容器中
        self.horizontalLayout_1.addWidget(win1)

        p1 = win1.addPlot(title="joint pos")  # 添加第一个绘图窗口
        p1.setLabel('left', text='pos/rad', color='#ffffff')  # y轴设置函数
        p1.showGrid(x=True, y=True)  # 栅格设置函数
        p1.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p1.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p1.addLegend(size=(50, 30))  # 可选择是否添加legend
        return p1

    # 绘画关节角和关节角速度曲线
    def plot_joint(self, t1, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)

    # 绘画关节角和关节角速度曲线
    def plot_force(self, t2, f):
        # 绘制力图
        self.p1.plot(t2, f[:, 0], pen='b', name='F1', clear=True)
        self.p1.plot(t2, f[:, 1], pen='g', name='F2', clear=False)
        self.p1.plot(t2, f[:, 2], pen='r', name='F3', clear=False)
        self.p1.plot(t2, f[:, 3], pen='c', name='F4', clear=False)
        self.p1.plot(t2, f[:, 4], pen='m', name='F5', clear=False)
        self.p1.plot(t2, f[:, 5], pen='y', name='F6', clear=False)

    # 刷新按钮
    def refresh_radioButton(self):
        self.robot1_flag = self.radioButton_robot1.isChecked()
        self.robot2_flag = self.radioButton_robot2.isChecked()
        self.robot3_flag = self.radioButton_robot3.isChecked()
        self.all_flag = self.radioButton_all.isChecked()

        self.robot1_2_flag = self.radioButton_robot1_2.isChecked()
        self.robot2_2_flag = self.radioButton_robot2_2.isChecked()
        self.robot3_2q_flag = self.radioButton_robot3_2.isChecked()
        self.robot3_2f_flag = self.radioButton_robot3_4.isChecked()

        self.real_flag = self.radioButton_real.isChecked()
        self.gazebo_flag = self.radioButton_gazebo.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

    def go_init(self):
        self.init_flag = True
        self.move_flag = False
        self.home_flag = False

        # 先回到零位
        qq_init = np.array([0, -90, 0, -90, 0, 0.0]) * np.pi / 180

        if(self.read_pos_flag):
            if (self.robot1_flag or self.all_flag):
                # 获得规划起点
                qq_b1 = np.array(self.state_qq_list1[-1])

                # 调用规划函数
                [qq1_1, qv, qa] = gf.q_joint_space_plan_time(qq_b1, qq_init, self.T, self.t)
                [qq1_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, self.robot1_command_qq[0, :],
                                                             self.T, self.t)
                # 调用绘图函数
                k1_1 = len(qq1_1)
                k1_2 = len(qq1_2)
                k1 = k1_1 + k1_2
                t1 = np.linspace(0, self.T * (k1 - 1), k1)
                qq1 = np.zeros([k1, self.n])
                qq1[:k1_1, :] = qq1_1
                qq1[k1_1:, :] = qq1_2

                # 绘制关节角位置图
                self.plot_joint(t1, qq1)
                # 将规划好的位置定义为全局变量
                self.command_qq1_init = np.copy(qq1)

            if (self.robot2_flag or self.all_flag):
                # 获得规划起点
                qq_b2 = np.array(self.state_qq_list2[-1])
                # 调用规划函数
                [qq2_1, qv, qa] = gf.q_joint_space_plan_time(qq_b2, qq_init, self.T, self.t)
                [qq2_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, self.robot2_command_qq[0, :],
                                                             self.T, self.t)
                # 调用绘图函数
                k2_1 = len(qq2_1)
                k2_2 = len(qq2_2)
                k2 = k2_1 + k2_2
                t2 = np.linspace(0, self.T * (k2 - 1), k2)
                qq2 = np.zeros([k2, self.n])
                qq2[:k2_1, :] = qq2_1
                qq2[k2_1:, :] = qq2_2
                # 绘制关节角位置图
                self.plot_force(t2, qq2)
                # 将规划好的位置定义为全局变量
                self.command_qq2_init = np.copy(qq2)

            if (self.robot3_flag or self.all_flag):
                # 获得规划起点
                qq_b3 = np.array(self.state_qq_list3[-1])
                # 调用规划函数
                [qq3_1, qv, qa] = gf.q_joint_space_plan_time(qq_b3, qq_init, self.T, self.t)
                [qq3_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, self.robot3_command_qq[0, :],
                                                           self.T, self.t)
                # 调用绘图函数
                k3_1 = len(qq3_1)
                k3_2 = len(qq3_2)
                k3 = k3_1 + k3_2
                t3 = np.linspace(0, self.T * (k3 - 1), k3)
                qq3 = np.zeros([k3, self.n])
                qq3[:k3_1, :] = qq3_1
                qq3[k3_1:, :] = qq3_2
                # 绘制关节角位置图
                self.plot_force(t3, qq3)
                # 将规划好的位置定义为全局变量
                self.command_qq3_init = np.copy(qq3)

            msg = "运动到初始点已规划！\n"
            self.textEdit.setText(msg)
        else:
            msg = "请先读取规划数据！\n"
            self.textEdit.setText(msg)

    def go_move(self):
        self.init_flag = False
        self.move_flag = True
        self.home_flag = False

        msg = "已切换到搬运模式！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        self.init_flag = False
        self.move_flag = False
        self.home_flag = True

        # 获得规划终点
        qq_init = np.array([0, -90, 0, -90, 0, 0.0]) * np.pi / 180
        qq_home = np.array([0, 0, 0, 0, 0, 0, 0.0])

        # 调用规划函数
        if (self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq1_1, qv, qa] = gf.q_joint_space_plan_time(qq_b1, qq_init, self.T, self.t)
            [qq1_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, qq_home, self.T, self.t)
            # 调用绘图函数
            k1_1 = len(qq1_1)
            k1_2 = len(qq1_2)
            k1 = k1_1 + k1_2
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            qq1 = np.zeros([k1, self.n])
            qq1[:k1_1, :] = qq1_1
            qq1[k1_1:, :] = qq1_2
            # 绘制关节角位置图
            self.plot_joint(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_home = np.copy(qq1)

        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2_1, qv, qa] = gf.q_joint_space_plan_time(qq_b2, qq_init, self.T, self.t)
            [qq2_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, qq_home, self.T, self.t)
            # 调用绘图函数
            k2_1 = len(qq2_1)
            k2_2 = len(qq2_2)
            k2 = k2_1 + k2_2
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            qq2 = np.zeros([k2, self.n])
            qq2[:k2_1, :] = qq2_1
            qq2[k2_1:, :] = qq2_2
            # 绘制关节角位置图
            self.plot_force(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_home = np.copy(qq2)

        if (self.robot3_flag or self.all_flag):
            # 获得规划起点
            qq_b3 = np.array(self.state_qq_list3[-1])
            # 调用规划函数
            [qq3_1, qv, qa] = gf.q_joint_space_plan_time(qq_b3, qq_init, self.T, self.t)
            [qq3_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, qq_home, self.T, self.t)
            # 调用绘图函数
            k3_1 = len(qq3_1)
            k3_2 = len(qq3_2)
            k3 = k3_1 + k3_2
            t3 = np.linspace(0, self.T * (k3 - 1), k3)
            qq3 = np.zeros([k3, self.n])
            qq3[:k3_1, :] = qq3_1
            qq3[k3_1:, :] = qq3_2
            # 绘制关节角位置图
            self.plot_force(t3, qq3)
            # 将规划好的位置定义为全局变量
            self.command_qq3_home = np.copy(qq3)

        msg = "运动到家点已规划！\n"
        self.textEdit.setText(msg)

    #读取数据
    def read_pos(self):
        #读取数据
        if(self.robot1_flag or self.all_flag):
            pos_path1 = str(self.lineEdit_data_1.text())
            self.robot1_command_qq = fo.read(pos_path1)

        if(self.robot2_flag or self.all_flag):
            pos_path2 = str(self.lineEdit_data_2.text())
            self.robot2_command_qq = fo.read(pos_path2)

        if (self.robot3_flag or self.all_flag):
            pos_path3 = str(self.lineEdit_data_3.text())
            self.robot3_command_qq = fo.read(pos_path3)

            force_path3 = str(self.lineEdit_force_3.text())
            self.robot3_command_f = fo.read(force_path3)

        msg = "位置已读取！"
        self.read_pos_flag = True
        self.textEdit.setText(msg)

    #绘制数据图
    def plot_data(self):
        self.refresh_radioButton()
        if(not self.read_pos_flag):
            msg = "请先读取数据！"
            self.textEdit.setText(msg)
        else:
            # 读取数据
            if (self.robot1_2_flag):
                # 调用绘图函数
                k = len(self.robot1_command_qq)
                t = np.linspace(0, self.T * (k - 1), k)
                # 绘制关节角位置速度图
                self.plot_joint(t, self.robot1_command_qq)

            if (self.robot2_2_flag):
                # 调用绘图函数
                k = len(self.robot2_command_qq)
                t = np.linspace(0, self.T * (k - 1), k)
                # 绘制关节角位置速度图
                self.plot_joint(t, self.robot2_command_qq)

            if (self.robot3_2q_flag):
                # 调用绘图函数
                k = len(self.robot3_command_qq)
                t = np.linspace(0, self.T * (k - 1), k)
                # 绘制关节角位置速度图
                self.plot_joint(t, self.robot3_command_qq)

            if (self.robot3_2f_flag):
                # 调用绘图函数
                k = len(self.robot3_command_f)
                t = np.linspace(0, self.T * (k - 1), k)
                # 绘制关节角位置速度图
                self.plot_joint(t, self.robot3_command_f)

    ##关节角订阅回调函数
    def joint_callback1(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        #反馈关节有问题，改
        qq_r = np.copy(qq)
        qq_r[0] = qq[2]
        qq_r[2] = qq[0]
        self.state_t = self.state_t + self.T
        self.state_qq_list1.append(qq_r)
        self.state_qq1 = qq_r
        self.state_t_list.append(self.state_t)
        # 仅记录100个数据点
        del self.state_t_list[0]
        del self.state_qq_list1[0]

    def joint_callback2(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]
        qq_r = np.copy(qq)
        qq_r[0] = qq[2]
        qq_r[2] = qq[0]
        self.state_qq_list2.append(qq_r)
        self.state_qq2 = qq_r
        # 仅记录100个数据点
        del self.state_qq_list2[0]

    def joint_callback3(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]
        qq_r = np.copy(qq)
        qq_r[0] = qq[2]
        qq_r[2] = qq[0]
        self.state_qq_list3.append(qq_r)
        self.state_qq3 = qq_r
        # 仅记录100个数据点
        del self.state_qq_list3[0]

    ##关节角订阅回调函数
    def force_callback1(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list1.append(f)
        # 仅记录1000个数据点
        del self.state_f_list1[0]

    def force_callback2(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list2.append(f)
        # 仅记录1000个数据点
        del self.state_f_list2[0]

    def force_callback3(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list3.append(f)
        # 仅记录1000个数据点
        del self.state_f_list3[0]

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
        if (self.robot1_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list1)
            self.plot_joint(plot_t, plot_qq)

        if (self.robot2_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list2)
            self.plot_joint(plot_t, plot_qq)

        if (self.robot3_2q_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list3)
            self.plot_joint(plot_t, plot_qq)

        if (self.robot3_2f_flag):
            plot_t = np.array(self.state_t_list)
            plot_f = np.array(self.state_f_list3)
            self.plot_force(plot_t, plot_f)

    def run_topic(self):
        # 读取时间
        self.T = float(self.lineEdit_T.text())
        self.t = float(self.lineEdit_t.text())

        msg_time = "获取时间：\n" + "周期T：" + str(self.T) + \
                   "\n规划时长：" + str(self.t) + "\n"

        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos1_path, JointState, self.joint_callback1)
        rospy.Subscriber(self.sub_force1_path, WrenchStamped, self.force_callback1)
        self.pub1 = rospy.Publisher(self.pub1_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos2_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force2_path, WrenchStamped, self.force_callback2)
        self.pub2 = rospy.Publisher(self.pub2_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos3_path, JointState, self.joint_callback3)
        rospy.Subscriber(self.sub_force3_path, WrenchStamped, self.force_callback3)
        self.pub3 = rospy.Publisher(self.pub3_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"

        msg = msg_time + msg_tip
        self.textEdit.setText(msg)
        t1.start()

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        kk = 0
        command_qq_list1 = []
        command_qq_list2 = []
        command_qq_list3 = []

        # 求取数据长度
        if (self.robot1_flag):
            if(self.init_flag):
                kk = len(self.command_qq1_init)
                command_qq_list1 = self.command_qq1_init
            elif(self.home_flag):
                kk = len(self.command_qq1_home)
                command_qq_list1 = self.command_qq1_home
            else:
                kk = len(self.robot1_command_qq)
                command_qq_list1 = self.robot1_command_qq
        if (self.robot2_flag):
            if (self.init_flag):
                kk = len(self.command_qq2_init)
                command_qq_list2 = self.command_qq2_init
            elif (self.home_flag):
                kk = len(self.command_qq2_home)
                command_qq_list2 = self.command_qq2_home
            else:
                kk = len(self.robot2_command_qq)
                command_qq_list2 = self.robot2_command_qq
        if(self.robot3_flag):
            if (self.init_flag):
                kk = len(self.command_qq3_init)
                command_qq_list3 = self.command_qq3_init
            else:
                kk = len(self.command_qq3_home)
                command_qq_list3 = self.command_qq3_home

        if (self.all_flag):
            if (self.init_flag):
                kk = len(self.command_qq1_init)
                command_qq_list1 = self.command_qq1_init
                command_qq_list2 = self.command_qq2_init
                command_qq_list3 = self.command_qq3_init

            elif (self.home_flag):
                kk = len(self.command_qq1_home)
                command_qq_list1 = self.command_qq1_home
                command_qq_list2 = self.command_qq2_home
                command_qq_list3 = self.command_qq3_home
            else:
                kk = len(self.robot1_command_qq)
                command_qq_list1 = self.robot1_command_qq
                command_qq_list2 = self.robot2_command_qq
        # 进度条显示时间间隔
        show_time = int(kk * self.T * 10)

        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(show_time)

        # 设置绘图,用Qtimer开线程处理（线程4）
        self.timer_plot = QTimer()
        self.timer_plot.timeout.connect(self.realtime_plot)
        self.timer_plot.start(1000)

        # 发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                self.timer_p.stop()
                self.timer_plot.stop()
                break
            # 发送数据
            msg = "已发送机器人：\n"
            if (self.robot1_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = command_qq_list1[k, 0:self.n]
                else:
                    command_data.data = command_qq_list1[-1, 0:self.n]
                self.pub1.publish(command_data)
                if (k == 0):
                    msg = msg + "robot1已发送！\n"
            if (self.robot2_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = command_qq_list2[k, 0:self.n]
                else:
                    command_data.data = command_qq_list2[-1, 0:self.n]
                self.pub2.publish(command_data)
                if (k == 0):
                    msg = msg + "robot2已发送！\n"

            if(self.init_flag or self.home_flag):
                if (self.robot3_flag or self.all_flag):
                    command_data = Float64MultiArray()
                    if (k < kk):
                        command_data.data = command_qq_list3[k, 0:self.n]
                    else:
                        command_data.data = command_qq_list3[-1, 0:self.n]
                    self.pub3.publish(command_data)
                    if (k == 0):
                        msg = msg + "robot3已发送！\n"

            if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
                msg = "没有选中机械臂！\n"

            self.textEdit.setText(msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    def run_polish(self):
        # 运行标签启动
        self.run_flag = True

        kk = len(self.robot3_command_qq)
        # 发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                break
            # 发送数据
            msg = "已发送机器人：\n"
            msg = msg + "robot3已发送！\n"
            command_data = Float64MultiArray()
            if (k < kk):
                command_data.data = self.robot3_command_qq[k, :]
            else:
                command_data.data = self.robot3_command_qq[-1, :]
            self.pub3.publish(command_data)

            self.textEdit.setText(msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()


    # ===============窗口跳转函数================#
    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================手抓控制：双臂位置模式搬运物体================#
class UrsHandWindow1(QMainWindow, Ui_UrsHandForm1):
    # 建立全局变量
    state_qq_list1 = list(np.zeros([1000, 6]))
    state_f_list1 = list(np.zeros([1000, 6]))
    state_qq_list2 = list(np.zeros([1000, 6]))
    state_f_list2 = list(np.zeros([1000, 6]))
    state_dq_list1 = list(np.zeros(1000))
    state_dq_list2 = list(np.zeros(1000))
    state_t_list = list(np.zeros(1000))
    state_t = 0.0

    def __init__(self, parent=None):
        super(UrsHandWindow1, self).__init__(parent)
        self.T = 0.01
        self.t =10
        self.run_flag = True  # 开始或停止标签
        self.real_flag = False
        self.gazebo_flag = False

        self.robot1_flag = False
        self.robot2_flag = False
        self.all_flag = True

        self.robot1_2_flag = True
        self.robot2_2_flag = False

        self.init_flag = False
        self.move_flag = False
        self.home_flag = False

        self.state_qq1 = np.zeros(6)
        self.state_qq2 = np.zeros(6)

        self.sub_force1_path = "/robot1/ft_sensor_topic"
        self.sub_pos1_path = "/robot1/joint_states"
        self.pub1_path = "/robot1/ur5_position_controller/command"
        self.pub1_hand_path = "/robot1/finger_controller/command"
        self.sub_force2_path = "/robot2/ft_sensor_topic"
        self.sub_pos2_path = "/robot2/joint_states"
        self.pub2_path = "/robot2/ur5_position_controller/command"
        self.pub2_hand_path = "/robot2/finger_controller/command"

        self.n = 6  # 机械臂关节数

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        mainMenu = menubar.addMenu('&Main')

        # 文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window ', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        mainMenu.addAction(openMain)

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_read_data.clicked.connect(self.read_pos)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_refresh.clicked.connect(self.refresh_radioButton)
        self.button_work.clicked.connect(self.go_move)
        self.button_init_plan.clicked.connect(self.go_init)
        self.button_end_plan.clicked.connect(self.go_home)
        self.button_hand_on.clicked.connect(self.grasp_on)
        self.button_hand_off.clicked.connect(self.grasp_off)
        self.button_hand_stop.clicked.connect(self.grasp_stop)

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

    # 绘画关节角和关节角速度曲线
    def plot_force(self, t2, f):
        # 绘制力图
        self.p2.plot(t2, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t2, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t2, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t2, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t2, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t2, f[:, 5], pen='y', name='F6', clear=False)

    # 刷新按钮
    def refresh_radioButton(self):
        self.robot1_flag = self.radioButton_robot1.isChecked()
        self.robot2_flag = self.radioButton_robot2.isChecked()
        self.all_flag = self.radioButton_all.isChecked()

        self.robot1_2_flag = self.radioButton_robot1_2.isChecked()
        self.robot2_2_flag = self.radioButton_robot2_2.isChecked()

        self.real_flag = self.radioButton_real.isChecked()
        self.gazebo_flag = self.radioButton_gazebo.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

    def go_init(self):
        self.init_flag = True
        self.move_flag = False
        self.home_flag = False

        #先回到零位
        qq_init = np.array([0, -90, 0, -90, 0, 0.0])*np.pi/180

        if(self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])

            # 调用规划函数
            [qq1_1, qv, qa] = gf.q_joint_space_plan_time(qq_b1, qq_init, self.T, self.t)
            [qq1_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, self.robot1_command_qq[0, :],
                                                      self.T, self.t)
            # 调用绘图函数
            k1_1 = len(qq1_1)
            k1_2 = len(qq1_2)
            k1 = k1_1 + k1_2
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            qq1 = np.zeros([k1, self.n])
            qq1[:k1_1, :] = qq1_1
            qq1[k1_1:, :] = qq1_2

            # 绘制关节角位置图
            self.plot_joint(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_init = np.copy(qq1)

        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2_1, qv, qa] = gf.q_joint_space_plan_time(qq_b2, qq_init, self.T, self.t)
            [qq2_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, self.robot2_command_qq[0, :],
                                                      self.T, self.t)
            # 调用绘图函数
            k2_1 = len(qq2_1)
            k2_2 = len(qq2_2)
            k2 = k2_1 + k2_2
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            qq2 = np.zeros([k2, self.n])
            qq2[:k2_1, :] = qq2_1
            qq2[k2_1:, :] = qq2_2
            # 绘制关节角位置图
            self.plot_force(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_init = np.copy(qq2)

        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    def go_move(self):
        self.init_flag = False
        self.move_flag = True
        self.home_flag = False

        msg = "已切换到搬运模式！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        self.init_flag = False
        self.move_flag = False
        self.home_flag = True

        # 获得规划终点
        qq_home = np.array([0, 0, 0, 0, 0, 0, 0.0])
        qq_init = np.array([0, -90, 0, -90, 0, 0.0]) * np.pi / 180

        # 调用规划函数
        if (self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq1_1, qv, qa] = gf.q_joint_space_plan_time(qq_b1, qq_init, self.T, self.t)
            [qq1_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, qq_home, self.T, self.t)
            # 调用绘图函数
            k1_1 = len(qq1_1)
            k1_2 = len(qq1_2)
            k1 = k1_1 + k1_2
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            qq1 = np.zeros([k1, self.n])
            qq1[:k1_1, :] = qq1_1
            qq1[k1_1:, :] = qq1_2
            # 绘制关节角位置图
            self.plot_joint(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_home = np.copy(qq1)

        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2_1, qv, qa] = gf.q_joint_space_plan_time(qq_b2, qq_init, self.T, self.t)
            [qq2_2, qv, qa] = gf.q_joint_space_plan_time(qq_init, qq_home, self.T, self.t)
            # 调用绘图函数
            k2_1 = len(qq2_1)
            k2_2 = len(qq2_2)
            k2 = k2_1 + k2_2
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            qq2 = np.zeros([k2, self.n])
            qq2[:k2_1, :] = qq2_1
            qq2[k2_1:, :] = qq2_2
            # 绘制关节角位置图
            self.plot_force(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_home = np.copy(qq2)

        msg = "运动到家点已规划！\n"
        self.textEdit.setText(msg)

    #读取数据
    def read_pos(self):
        #读取数据
        if(self.robot1_flag or self.all_flag):
            pos_path1 = str(self.lineEdit_data_1.text())
            self.robot1_command_qq = fo.read(pos_path1)
            # 调用绘图函数
            k = len(self.robot1_command_qq)
            t = np.linspace(0, self.T * (k - 1), k)
            # 绘制关节角位置速度图
            self.plot_joint(t, self.robot1_command_qq)

        if(self.robot2_flag or self.all_flag):
            pos_path2 = str(self.lineEdit_data_2.text())
            self.robot2_command_qq = fo.read(pos_path2)
            # 调用绘图函数
            k = len(self.robot2_command_qq)
            t = np.linspace(0, self.T * (k - 1), k)
            # 绘制关节角位置速度图
            self.plot_force(t, self.robot2_command_qq)



        msg = "位置已读取！"
        self.read_pos_flag = True
        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback1(self, msg):
        qq = np.zeros(self.n)
        #调整关节角
        qq[0] = msg.position[4]
        qq[1] = msg.position[3]
        qq[2] = msg.position[0]
        qq[3] = msg.position[5]
        qq[4] = msg.position[6]
        qq[5] = msg.position[7]

        #手抓
        dq = msg.position[1]

        self.state_t = self.state_t + self.T
        self.state_qq_list1.append(qq)
        self.state_qq1 = qq
        self.state_t_list.append(self.state_t)
        self.state_dq_list1.append(dq)

        # 仅记录100个数据点
        del self.state_t_list[0]
        del self.state_qq_list1[0]
        del self.state_dq_list1[0]

    def joint_callback2(self, msg):
        qq = np.zeros(self.n)
        # 调整关节角
        qq[0] = msg.position[4]
        qq[1] = msg.position[3]
        qq[2] = msg.position[0]
        qq[3] = msg.position[5]
        qq[4] = msg.position[6]
        qq[5] = msg.position[7]

        # 手抓
        dq = msg.position[1]

        self.state_qq_list2.append(qq)
        self.state_qq2 = qq
        self.state_dq_list2.append(dq)

        # 仅记录100个数据点
        del self.state_qq_list2[0]
        del self.state_dq_list2[0]

    ##关节角订阅回调函数
    def force_callback1(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list1.append(f)
        # 仅记录1000个数据点
        del self.state_f_list1[0]

    def force_callback2(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        # 存储数据
        self.state_f_list2.append(f)
        # 仅记录1000个数据点
        del self.state_f_list2[0]

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
        if (self.robot1_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list1)
            plot_f = np.array(self.state_f_list1)
            self.plot_joint(plot_t, plot_qq)
            self.plot_force(plot_t, plot_f)

        if (self.robot2_2_flag):
            plot_t = np.array(self.state_t_list)
            plot_qq = np.array(self.state_qq_list2)
            plot_f = np.array(self.state_f_list2)
            self.plot_joint(plot_t, plot_qq)
            self.plot_force(plot_t, plot_f)

    def run_topic(self):
        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos1_path, JointState, self.joint_callback1)
        rospy.Subscriber(self.sub_force1_path, WrenchStamped, self.force_callback1)
        self.pub1 = rospy.Publisher(self.pub1_path, Float64MultiArray, queue_size=100)
        self.pub1_hand = rospy.Publisher(self.pub1_hand_path, Float64, queue_size=100)

        rospy.Subscriber(self.sub_pos2_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force2_path, WrenchStamped, self.force_callback2)
        self.pub2 = rospy.Publisher(self.pub2_path, Float64MultiArray, queue_size=100)
        self.pub2_hand = rospy.Publisher(self.pub2_hand_path, Float64, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"

        msg = msg_tip
        self.textEdit.setText(msg)
        t1.start()

    def grasp_on(self):
        dq1 = self.state_dq_list1[-1]
        dq2 = 0.001*float(self.lineEdit_F.text())
        num = 100
        dq = np.linspace(dq1, dq2, num)
        # 发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            if (k >= num):
                break
            self.pub1_hand.publish(dq[k])
            self.pub2_hand.publish(dq[k])
            k = k + 1
            rate.sleep()

    def grasp_off(self):
        dq1 = self.state_dq_list1[-1]
        dq2 = 0.040
        num = 100
        dq = np.linspace(dq1, dq2, num)
        # 发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            if(k>=num):
                break
            self.pub1_hand.publish(dq[k])
            self.pub2_hand.publish(dq[k])
            k = k+1
            rate.sleep()

    def grasp_stop(self):
        dq1 = self.state_dq_list1[-1]
        dq2 = 0.0
        num = 100
        dq = np.linspace(dq1, dq2, num)
        # 发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            if (k >= num):
                break
            self.pub1_hand.publish(dq[k])
            self.pub2_hand.publish(dq[k])
            k = k + 1
            rate.sleep()

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        kk = 0
        command_qq_list1 = []
        command_qq_list2 = []

        # 求取数据长度
        if (self.robot1_flag):
            if(self.init_flag):
                kk = len(self.command_qq1_init)
                command_qq_list1 = self.command_qq1_init
            elif(self.home_flag):
                kk = len(self.command_qq1_home)
                command_qq_list1 = self.command_qq1_home
            else:
                kk = len(self.robot1_command_qq)
                command_qq_list1 = self.robot1_command_qq
        if (self.robot2_flag):
            if (self.init_flag):
                kk = len(self.command_qq2_init)
                command_qq_list2 = self.command_qq2_init
            elif (self.home_flag):
                kk = len(self.command_qq2_home)
                command_qq_list2 = self.command_qq2_home
            else:
                kk = len(self.robot2_command_qq)
                command_qq_list2 = self.robot2_command_qq
        if (self.all_flag):
            if (self.init_flag):
                kk = len(self.command_qq1_init)
                command_qq_list1 = self.command_qq1_init
                command_qq_list2 = self.command_qq2_init
            elif (self.home_flag):
                kk = len(self.command_qq1_home)
                command_qq_list1 = self.command_qq1_home
                command_qq_list2 = self.command_qq2_home
            else:
                kk = len(self.robot1_command_qq)
                command_qq_list1 = self.robot1_command_qq
                command_qq_list2 = self.robot2_command_qq
        # 进度条显示时间间隔
        show_time = int(kk * self.T * 10)

        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(show_time)

        # 设置绘图,用Qtimer开线程处理（线程4）
        self.timer_plot = QTimer()
        self.timer_plot.timeout.connect(self.realtime_plot)
        self.timer_plot.start(1000)

        # 发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                self.timer_p.stop()
                self.timer_plot.stop()
                break
            # 发送数据
            msg = "已发送机器人：\n"
            if (self.robot1_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = command_qq_list1[k, 0:self.n]
                else:
                    command_data.data = command_qq_list1[-1, 0:self.n]
                self.pub1.publish(command_data)
                if (k == 0):
                    msg = msg + "robot1已发送！\n"
            if (self.robot2_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = command_qq_list2[k, 0:self.n]
                else:
                    command_data.data = command_qq_list2[-1, 0:self.n]
                self.pub2.publish(command_data)
                if (k == 0):
                    msg = msg + "robot2已发送！\n"

            if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
                msg = "没有选中机械臂！\n"

            self.textEdit.setText(msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    # ===============窗口跳转函数================#
    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================两个自制机械臂和UR5的协同控制：阻抗+位置搬运控制================#
class ArmctrWindow1(QMainWindow, Ui_ArmctrForm1):
    # 建立全局变量
    state_t = 0.0
    def __init__(self, parent=None):
        super(ArmctrWindow1, self).__init__(parent)
        self.T = 0.01
        self.t = 10
        self.run_flag = False  # 开始或停止标签
        self.real_flag = False

        self.armc_flag = False
        self.armt_flag = False
        self.armr_flag = False
        self.all_flag = False

        self.force_flag = False

        self.read_pos_flag = False

        self.init_flag = False
        self.data_flag = False
        self.home_flag = False

        self.sub_force1_path = "/robot1/ft_sensor_topic"
        self.sub_pos1_path = "/robot1/joint_states"
        self.pub1_path = "/robot1/ur5_position_controller/command"
        self.sub_force2_path = "/robot2/ft_sensor_topic"
        self.sub_pos2_path = "/robot2/joint_states"
        self.pub2_path = "/robot2/armt_position_controller/command"
        self.sub_force3_path = "/robot3/ft_sensor_topic"
        self.sub_pos3_path = "/robot3/joint_states"
        self.pub3_path = "/robot3/armc_position_controller/command"


        self.n1 = 6  # 机械臂关节数
        self.n2 = 7
        self.n3 = 7

        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&打开')
        mainMenu = menubar.addMenu('&返回主界面')

        # 文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), '主窗口', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('返回主窗口')
        openMain.triggered.connect(self.gotoMain)
        mainMenu.addAction(openMain)

        # -------------------文件菜单-------------------#
        # 中打开文件操作
        openFile = QAction(QIcon('exit.png'), '打开文件', self)
        openFile.setShortcut('Ctrl+o')
        openFile.setStatusTip('打开存数据的文件')
        openFile.triggered.connect(self.fileOpen)
        fileMenu.addAction(openFile)

        # 文件菜单中关闭操作
        exitAction = QAction(QIcon('exit.png'), '&关闭', self)
        exitAction.setShortcut('Ctrl+q')
        exitAction.setStatusTip('退出软件')
        exitAction.triggered.connect(qApp.quit)
        fileMenu.addAction(exitAction)

        # =======================绘图相关设置=======================#
        self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_read.clicked.connect(self.read_data)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_plan_init.clicked.connect(self.go_init)
        self.button_plan_data.clicked.connect(self.run_data)
        self.button_plan_home.clicked.connect(self.go_home)
        self.checkBox_real.stateChanged.connect(self.gazebo_or_real)
        self.checkBox_force.stateChanged.connect(self.force_or_pos)

    # ===============按钮功能模块相关函数================#
    # 采用pyqtgraph绘制曲线,添加画板
    def set_graph_ui(self):
        pg.setConfigOptions(antialias=True)  # pg全局变量设置函数，antialias=True开启曲线抗锯齿

        self.win1 = pg.GraphicsLayoutWidget()  # 创建pg layout，可实现数据界面布局自动管理
        self.win2 = pg.GraphicsLayoutWidget()
        self.win3 = pg.GraphicsLayoutWidget()

        # pg绘图窗口可以作为一个widget添加到GUI中的graph_layout，当然也可以添加到Qt其他所有的容器中
        self.horizontalLayout_1.addWidget(self.win1)
        self.horizontalLayout_2.addWidget(self.win2)
        self.horizontalLayout_3.addWidget(self.win3)
        self.force_or_pos()

    # 打开文件的地址和内容
    def fileOpen(self):
        # 打开文件操作
        path = os.path.join(os.getcwd(), '../', 'data/robots/armctr')
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

    #选择位置或六维力,设置对应画板
    def force_or_pos(self):
        self.win1.clear()
        self.win2.clear()
        self.win3.clear()
        self.force_flag = self.checkBox_force.isChecked()
        if(self.force_flag ):
            p1 = self.win1.addPlot(title="UR5_末端力")  # 添加第一个绘图窗口
            p1.setLabel('left', text='力/N', color='#ffffff')  # y轴设置函数
            p1.showGrid(x=True, y=True)  # 栅格设置函数
            p1.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
            p1.setLabel('bottom', text='时间', units='s')  # x轴设置函数
            p1.addLegend(size=(50, 30))

            p2 = self.win2.addPlot(title="Armt_末端力")  # 添加第一个绘图窗口
            p2.setLabel('left', text='力/N', color='#ffffff')  # y轴设置函数
            p2.showGrid(x=True, y=True)  # 栅格设置函数
            p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
            p2.setLabel('bottom', text='时间', units='s')  # x轴设置函数
            p2.addLegend(size=(50, 30))

            p3 = self.win3.addPlot(title="Armc_末端力")  # 添加第一个绘图窗口
            p3.setLabel('left', text='力/N', color='#ffffff')  # y轴设置函数
            p3.showGrid(x=True, y=True)  # 栅格设置函数
            p3.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
            p3.setLabel('bottom', text='时间', units='s')  # x轴设置函数
            p3.addLegend(size=(50, 30))

            self.p1 = p1
            self.p2 = p2
            self.p3 = p3

            msg = "选择绘制六维力"
            self.textEdit.setText(msg)
        else:
            p1 = self.win1.addPlot(title="UR5_关节位置")  # 添加第一个绘图窗口
            p1.setLabel('left', text='位置/rad', color='#ffffff')  # y轴设置函数
            p1.showGrid(x=True, y=True)  # 栅格设置函数
            p1.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
            p1.setLabel('bottom', text='时间', units='s')  # x轴设置函数
            p1.addLegend(size=(50, 30))  # 可选择是否添加legend

            p2 = self.win2.addPlot(title="Armt_关节位置")  # 添加第一个绘图窗口
            p2.setLabel('left', text='位置/rad', color='#ffffff')  # y轴设置函数
            p2.showGrid(x=True, y=True)  # 栅格设置函数
            p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
            p2.setLabel('bottom', text='时间', units='s')  # x轴设置函数
            p2.addLegend(size=(50, 30))  # 可选择是否添加legend

            p3 = self.win3.addPlot(title="Armc_关节位置")  # 添加第一个绘图窗口
            p3.setLabel('left', text='位置/rad', color='#ffffff')  # y轴设置函数
            p3.showGrid(x=True, y=True)  # 栅格设置函数
            p3.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
            p3.setLabel('bottom', text='时间', units='s')  # x轴设置函数
            p3.addLegend(size=(50, 30))  # 可选择是否添加legend

            self.p1 = p1
            self.p2 = p2
            self.p3 = p3

            msg = "选择绘制位置"
            self.textEdit.setText(msg)

    # 绘画关节角和关节角速度曲线
    def plot_joint1(self, t1, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)

    def plot_joint2(self, t, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p2.plot(t, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p2.plot(t, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p2.plot(t, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p2.plot(t, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p2.plot(t, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p2.plot(t, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p2.plot(t, qq[:, 6], pen='k', name='qq7', clear=False)

    def plot_joint3(self, t, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p2.plot(t, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p2.plot(t, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p2.plot(t, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p2.plot(t, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p2.plot(t, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p2.plot(t, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p2.plot(t, qq[:, 6], pen='k', name='qq7', clear=False)

    def plot_force1(self, t, f):
        # 绘制速度图
        self.p1.plot(t, f[:, 0], pen='b', name='F1', clear=True)
        self.p1.plot(t, f[:, 1], pen='g', name='F2', clear=False)
        self.p1.plot(t, f[:, 2], pen='r', name='F3', clear=False)
        self.p1.plot(t, f[:, 3], pen='c', name='F4', clear=False)
        self.p1.plot(t, f[:, 4], pen='m', name='F5', clear=False)
        self.p1.plot(t, f[:, 5], pen='y', name='F6', clear=False)

    def plot_force2(self, t, f):
        # 绘制速度图
        self.p2.plot(t, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t, f[:, 5], pen='y', name='F6', clear=False)

    def plot_force3(self, t, f):
        # 绘制速度图
        self.p3.plot(t, f[:, 0], pen='b', name='F1', clear=True)
        self.p3.plot(t, f[:, 1], pen='g', name='F2', clear=False)
        self.p3.plot(t, f[:, 2], pen='r', name='F3', clear=False)
        self.p3.plot(t, f[:, 3], pen='c', name='F4', clear=False)
        self.p3.plot(t, f[:, 4], pen='m', name='F5', clear=False)
        self.p3.plot(t, f[:, 5], pen='y', name='F6', clear=False)

    def gazebo_or_real(self):
        self.real_flag = self.checkBox_real.isChecked()
        if (self.real_flag):
            self.sub_force1_path = "/armr/ft_sensor_topic"
            self.sub_pos1_path = "/armr/joint_states"
            self.pub1_path = "/armr/joint_command"
            self.sub_force2_path = "/armt/ft_sensor_topic"
            self.sub_pos2_path = "/armt/joint_states"
            self.pub2_path = "/armt/joint_command"
            self.sub_force3_path = "/armc/ft_sensor_topic"
            self.sub_pos3_path = "/joint_states"
            self.pub3_path = "/all_joints_position_group_controller/command"
            msg = "选择实物"
            self.textEdit.setText(msg)
        else:
            self.sub_force1_path = "/robot1/ft_sensor_topic"
            self.sub_pos1_path = "/robot1/joint_states"
            self.pub1_path = "/robot1/armr_position_controller/command"
            self.sub_force2_path = "/robot2/ft_sensor_topic"
            self.sub_pos2_path = "/robot2/joint_states"
            self.pub2_path = "/robot2/armt_position_controller/command"
            self.sub_force3_path = "/robot3/ft_sensor_topic"
            self.sub_pos3_path = "/robot3/joint_states"
            self.pub3_path = "/robot3/armc_position_controller/command"
            msg = "选择仿真"
            self.textEdit.setText(msg)

    # 刷新按钮
    def refresh_radioButton(self):
        self.armr_flag = self.radioButton_armr.isChecked()
        self.armt_flag = self.radioButton_armt.isChecked()
        self.armc_flag = self.radioButton_armc.isChecked()
        self.all_flag = self.radioButton_all.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

    # 读取给定关节角度
    def read_data(self):
        # 读取数据
        armt_pos_path = str(self.lineEdit_qq_armt.text())
        armc_pos_path = str(self.lineEdit_qq_armc.text())
        armr_pos_path = str(self.lineEdit_qq_armr.text())
        armt_f_path = str(self.lineEdit_f_armt.text())
        armc_f_path = str(self.lineEdit_f_armc.text())
        armr_f_path = str(self.lineEdit_f_armr.text())

        #专门读取格式:FileOpen.read_xx()
        self.command_qq1_data = fo.read(armr_pos_path)
        self.command_qq2_data = fo.read(armt_pos_path)
        self.command_qq3_data = fo.read(armc_pos_path)
        self.command_f1_data = fo.read(armr_f_path)
        self.command_f2_data = fo.read(armt_f_path)
        self.command_f3_data = fo.read(armc_f_path)

        # 调用绘图函数
        self.plot_plan()

        msg = "位置已读取！"
        self.read_pos_flag = True
        self.textEdit.setText(msg)

        self.textEdit.setText(msg)

    #绘制规划图
    def plot_plan(self):
        self.force_flag = self.checkBox_force.isChecked()
        if(self.force_flag):

            k1 = len(self.command_f1_data)
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            self.plot_force1(t1, self.command_f1_data)

            k2 = len(self.command_f2_data)
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            self.plot_force2(t2, self.command_f2_data)

            k3 = len(self.command_f3_data)
            t3 = np.linspace(0, self.T * (k3 - 1), k3)
            self.plot_force3(t3, self.command_f3_data)
        else:
            k1 = len(self.command_qq1_data)
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            self.plot_joint1(t1, self.command_qq1_data)

            k2 = len(self.command_qq2_data)
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            self.plot_joint2(t2, self.command_qq2_data)

            k3 = len(self.command_qq3_data)
            t3 = np.linspace(0, self.T * (k3 - 1), k3)
            self.plot_joint3(t3, self.command_qq3_data)

    # 运动到初始位置
    def go_init(self):
        #自动刷新选择按钮
        self.refresh_radioButton()

        self.run_flag = False
        self.init_flag = True
        self.data_flag = False
        self.home_flag = False
        if (not self.read_pos_flag):
            msg = "请先读取规划数据！"
            self.textEdit.setText(msg)
            return -1

        msg = ''
        if (self.armr_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = self.state_qq1
            # 调用规划函数
            [qq1, _, _] = gf.q_joint_space_plan_time(qq_b1, self.command_qq1_data[0, :],
                                                     self.T, self.t)
            # 调用绘图函数
            k1 = len(qq1[:, 0])
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            self.plot_joint1(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_init = np.copy(qq1)
            msg = msg + "armt运动到初始点已规划！\n"

        if (self.armt_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = self.state_qq2
            # 调用规划函数
            [qq2, _, _] = gf.q_joint_space_plan_time(qq_b2, self.command_qq2_data[0, :],
                                                     self.T, self.t)
            # 调用绘图函数
            k2 = len(qq2[:, 0])
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            self.plot_joint2(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_init = np.copy(qq2)
            msg = msg + "armt运动到初始点已规划！\n"

        if (self.armc_flag or self.all_flag):
            # 获得规划起点
            qq_b3 = self.state_qq3
            # 调用规划函数
            [qq3, _, _] = gf.q_joint_space_plan_time(qq_b3, self.command_qq3_data[0, :],
                                                     self.T, self.t)
            # 调用绘图函数
            k3 = len(qq3[:, 0])
            t3 = np.linspace(0, self.T * (k3 - 1), k3)
            self.plot_joint3(t3, qq3)
            # 将规划好的位置定义为全局变量
            self.command_qq3_init = np.copy(qq3)
            msg = msg + "armt运动到初始点已规划！\n"

        self.textEdit.setText(msg)

    def run_data(self):
        # 自动刷新选择按钮
        self.refresh_radioButton()

        self.run_flag = False
        self.init_flag = False
        self.data_flag = True
        self.home_flag = False

        msg = "已切换到阻抗数据轨迹段！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        # 自动刷新选择按钮
        self.refresh_radioButton()

        self.run_flag = False
        self.init_flag = False
        self.data_flag = False
        self.home_flag = True

        # 获得规划起点
        qq_home = np.array([0, 0, 0, 0, 0, 0, 0.0])

        msg = ''
        if (self.armr_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = self.state_qq1
            # 调用规划函数
            [qq1, _, _] = gf.q_joint_space_plan_time(qq_b1, qq_home, self.T, self.t)
            # 调用绘图函数
            k1 = len(qq1[:, 0])
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            self.plot_joint1(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_home = np.copy(qq1)
            msg = msg + "armt运动到home点已规划！\n"

        if (self.armt_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = self.state_qq2
            # 调用规划函数
            [qq2, _, _] = gf.q_joint_space_plan_time(qq_b2, qq_home, self.T, self.t)
            # 调用绘图函数
            k2 = len(qq2[:, 0])
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
            self.plot_joint2(t2, qq2)
            # 将规划好的位置定义为全局变量
            self.command_qq2_home = np.copy(qq2)
            msg = msg + "armt运动到home点已规划！\n"

        if (self.armc_flag or self.all_flag):
            # 获得规划起点
            qq_b3 = self.state_qq3
            # 调用规划函数
            [qq3, _, _] = gf.q_joint_space_plan_time(qq_b3, qq_home, self.T, self.t)
            # 调用绘图函数
            k3 = len(qq3[:, 0])
            t3 = np.linspace(0, self.T * (k3 - 1), k3)
            self.plot_joint3(t3, qq3)
            # 将规划好的位置定义为全局变量
            self.command_qq3_home = np.copy(qq3)
            msg = msg + "armc运动到home点已规划！\n"

        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback1(self, msg):
        qq = np.zeros(self.n1)
        for i in range(self.n1):
            qq[i] = msg.position[i]

        self.state_t = self.state_t + self.T
        self.state_qq1 = qq

    def joint_callback2(self, msg):
        qq = np.zeros(self.n2)
        for i in range(self.n2):
            qq[i] = msg.position[i]

        self.state_qq2 = qq

    def joint_callback3(self, msg):
        qq = np.zeros(self.n3)
        for i in range(self.n3):
            qq[i] = msg.position[i]

        self.state_qq3 = qq

    ##末端力订阅回调函数
    def force_callback1(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        self.state_f1 = f

    def force_callback2(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        self.state_f2 = f

    def force_callback3(self, msg):
        f = np.zeros(6)
        f[0] = msg.wrench.force.x
        f[1] = msg.wrench.force.y
        f[2] = msg.wrench.force.z
        f[3] = msg.wrench.torque.x
        f[4] = msg.wrench.torque.y
        f[5] = msg.wrench.torque.z
        self.state_f3 = f

    ##回调线程
    def thread_spin(self):
        rospy.spin()

    def probar_show(self):
        self.step_p = self.step_p + 1
        self.progressBar.setValue(self.step_p)
        if (self.step_p > 99):
            self.timer_p.stop()

    def stop(self):
        self.run_flag = False

    def run_topic(self):
        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos1_path, JointState, self.joint_callback1)
        rospy.Subscriber(self.sub_force1_path, WrenchStamped, self.force_callback1)
        self.pub1 = rospy.Publisher(self.pub1_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos2_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force2_path, WrenchStamped, self.force_callback2)
        self.pub2 = rospy.Publisher(self.pub2_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos3_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force3_path, WrenchStamped, self.force_callback2)
        self.pub3 = rospy.Publisher(self.pub3_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"

        msg = msg_tip
        self.textEdit.setText(msg)
        t1.start()

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        # 提示标语
        msg = "开始下发命令！\n"
        self.textEdit.setText(msg)

        # 创建阻抗自适应阻抗实例
        imp_arm1 = imp.IIMPController_iter_TCP()

        # 获取阻抗参数
        self.M = np.array([0.0, 0.0, 4.0, 0.0, 0.0, 0.0])
        self.B = np.array([0.0, 0.0, 800.0, 0.0, 0.0, 0.0])
        self.K = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.I = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # 输入参数
        robot = 'armc'
        [DH0, q_max, q_min] = gf.get_robot_parameter(robot)
        imp_arm1.get_robot_parameter(DH0, q_max, q_min)
        imp_arm1.get_period(self.T)
        # 实时调整阻抗参数
        imp_arm1.get_imp_parameter(self.M, self.B, self.K, self.I)

        if (not self.real_flag):
            # 设置绘图,用Qtimer开线程处理（线程4）
            self.timer_plot = QTimer()
            self.timer_plot.timeout.connect(self.realtime_plot)
            self.timer_plot.start(1000)

        # 发送关节角度
        command_qq1 = 0
        command_qq2 = 0

        if (self.robot1_flag or self.all_flag):
            if (self.init_flag):
                command_qq1 = np.copy(self.command_qq1_init)
            if (self.data_flag):
                command_qq1 = np.copy(self.command_qq1_data)
            if (self.home_flag):
                command_qq1 = np.copy(self.command_qq1_home)

        if (self.robot2_flag or self.all_flag):
            if (self.init_flag):
                command_qq2 = np.copy(self.command_qq2_init)
            if (self.data_flag):
                command_qq2 = np.copy(self.command_qq2_data)
            if (self.home_flag):
                command_qq2 = np.copy(self.command_qq2_home)

        if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
            msg = '没有选择机械臂，请选择机械臂并刷新按钮！'
            self.textEdit.setText(msg)
            return -1

        kk = len(command_qq1)
        k = 0
        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(10 * self.T * kk)

        rate = rospy.Rate(100)
        msg = ''
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                if (not self.real_flag):
                    self.timer_plot.stop()
                    self.timer_p.stop()
                break

            if (self.robot1_flag or self.all_flag):
                command_data = Float64MultiArray()
                if (k < kk):
                    command_data.data = command_qq1[k, :]
                else:
                    break
                self.pub1.publish(command_data)
                if (k == 0):
                    msg = msg + "armt开始运动!\n"

            if (self.robot2_flag or self.all_flag):
                command_data = Float64MultiArray()
                if(self.data_flag):
                    if (k < kk):
                        qq = command_qq2[k, :]
                        fd = self.command_f2_data[k, :]
                    else:
                        qq = command_qq2[-1, :]
                        fd = self.command_f2_data[-1, :]
                    imp_arm1.get_expect_joint(qq)
                    imp_arm1.get_current_joint(np.array(self.state_qq_list2[-1]))
                    # 读取当前关节角和力
                    imp_arm1.get_expect_force(fd)
                    imp_arm1.get_current_force(np.array(self.state_f_list2[-1]))
                    # 计算修正关节角
                    qr = imp_arm1.compute_imp_joint()
                    # 发送数据

                    command_data.data = qr
                else:
                    if (k < kk):
                        command_data.data = command_qq2[k, :]
                    else:
                        break

                self.pub2.publish(command_data)
                if (k == 0):
                    msg = msg + "armc开始运动!\n"

            if (k == 0):
                self.textEdit.setText(msg)
            k = k + 1
            QApplication.processEvents()

            rate.sleep()
        msg = "运行完成！"
        self.textEdit.setText(msg)

    # ===============窗口跳转函数================#
    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()
