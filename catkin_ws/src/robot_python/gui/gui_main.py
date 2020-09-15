#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于GUI开发界面函数
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2020.4.21
#系统函数
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
from main_windon import Ui_MainWindow
from armc_form1 import Ui_ArmcForm1
from armc_form2 import Ui_ArmcForm2
from urs_form1 import Ui_UrsForm1
from urs_form2 import Ui_UrsForm2
from urs_form3 import Ui_UrsForm3
from urs_hand_form1 import Ui_UrsHandForm1
from armct_form1 import Ui_ArmctForm1
from armct_form2 import Ui_ArmctForm2
from armct_form3 import Ui_ArmctForm3
from impedance_form1 import Ui_ImpForm1
from impedance_form2 import Ui_ImpForm2
from impedance_form3 import Ui_ImpForm3
from circularPlan_form1 import Ui_CirForm1
from linePlan_form1 import Ui_LineForm1
from rundata_form import Ui_RunForm
from test_form1 import Ui_TestForm1
from teaching_form1 import Ui_TeachingForm1
from technology_form1 import Ui_TechnologyForm1

#自定义文件
import gui_function as gf
from robot_python import ImpedanceControl as imp
from robot_python import TeachingLearning as tl
from robot_python import FileOpen as fo
from robot_python import Kinematics as kin
from robot_python import PathPlan as pap

#**********************************主窗口***************************************#
class MainWindow(QMainWindow, Ui_MainWindow):
    #建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.T = 0.01
        self.run_flag = True # 开始或停止标签
        self.arm_flag = False
        self.real_flag = False
        self.sub_force_path = "/ft_sensor_topic"
        self.sub_pos_path = "/joint_states"
        self.pub_path = "/armc/joint_positions_controller/command"
        self.n = 7  # 机械臂关节数

        self.setupUi(self)
        self.initUI()
    def initUI(self):
        # ======================菜单栏功能模块=======================#
        #创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        armcMenu = menubar.addMenu('&Armc')
        ursMenu = menubar.addMenu('&3URs')
        handMenu = menubar.addMenu('&Hand')
        armctMenu = menubar.addMenu('&Armc_Armt')
        planMenu = menubar.addMenu('&Plan')
        impedanceMenu = menubar.addMenu('Impedance')
        teachMenu = menubar.addMenu('&Teaching')
        demoMenu = menubar.addMenu('&Technology')
        testMenu = menubar.addMenu('&Test')
        helpMenu = menubar.addMenu('&Help')

        #-------------------文件菜单-------------------#
        # 中打开文件操作
        openFile = QAction(QIcon('exit.png'), 'Open', self)
        openFile.setShortcut('Ctrl+o')
        openFile.setStatusTip('Open new File')
        openFile.triggered.connect(self.fileOpen)
        fileMenu.addAction(openFile)

        #文件菜单中关闭操作
        exitAction = QAction(QIcon('exit.png'), '&Exit', self)
        exitAction.setShortcut('Ctrl+q')
        exitAction.setStatusTip('Exit application')
        exitAction.triggered.connect(qApp.quit)
        fileMenu.addAction(exitAction)

        # -------------------Armc菜单-------------------#
        #armc机械臂关节空间规划
        openArmc1 = QAction(QIcon('exit.png'), 'armc joint plan', self)
        openArmc1.setShortcut('Ctrl+c')
        openArmc1.setStatusTip('Open armc joint plan form')
        openArmc1.triggered.connect(self.gotoArmc1)
        armcMenu.addAction(openArmc1)

        # armc机械臂增量规划
        openArmc2 = QAction(QIcon('exit.png'), 'armc inc plan', self)
        openArmc2.setShortcut('Ctrl+c')
        openArmc2.setStatusTip('Open armc inc plan form')
        openArmc2.triggered.connect(self.gotoArmc2)
        armcMenu.addAction(openArmc2)

        # -------------------URs菜单-------------------#
        #3个UR机械臂关节空间规划
        openUrs1 = QAction(QIcon('exit.png'), 'Open URs joint plan', self)
        openUrs1.setShortcut('Ctrl+c')
        openUrs1.setStatusTip('Open URs joint plan form')
        openUrs1.triggered.connect(self.gotoUrs1)
        ursMenu.addAction(openUrs1)

        #2个UR机械臂搬运物体
        openUrs2 = QAction(QIcon('exit.png'), 'Open URs move object', self)
        openUrs2.setShortcut('Ctrl+c')
        openUrs2.setStatusTip('Open URs move object form')
        openUrs2.triggered.connect(self.gotoUrs2)
        ursMenu.addAction(openUrs2)

        # 3个UR机械臂初始位置调整
        openUrs3 = QAction(QIcon('exit.png'), 'Open URs go init', self)
        openUrs3.setShortcut('Ctrl+c')
        openUrs3.setStatusTip('Open URs go init form')
        openUrs3.triggered.connect(self.gotoUrs3)
        ursMenu.addAction(openUrs3)

        # -------------------hand菜单-------------------#
        #URs加手抓
        openHand1 = QAction(QIcon('exit.png'), 'Open Hand URs', self)
        openHand1.setShortcut('Ctrl+c')
        openHand1.setStatusTip('Open Hand URs joint plan form')
        openHand1.triggered.connect(self.gotoHand1)
        handMenu.addAction(openHand1)

        # -------------------armct菜单-------------------#
        openArmct1 = QAction(QIcon('exit.png'), 'joint plan form', self)
        openArmct1.setShortcut('Ctrl+c')
        openArmct1.setStatusTip('关节空间规划')
        openArmct1.triggered.connect(self.gotoArmct1)
        armctMenu.addAction(openArmct1)

        openArmct2 = QAction(QIcon('exit.png'), 'cartesian plan form', self)
        openArmct2.setShortcut('Ctrl+c')
        openArmct2.setStatusTip('笛卡尔空间规划')
        openArmct2.triggered.connect(self.gotoArmct2)
        armctMenu.addAction(openArmct2)

        openArmct3 = QAction(QIcon('exit.png'), 'imp plan form', self)
        openArmct3.setShortcut('Ctrl+c')
        openArmct3.setStatusTip('笛卡尔空间规划')
        openArmct3.triggered.connect(self.gotoArmct3)
        armctMenu.addAction(openArmct3)

        #规划菜单栏:打开圆规划
        openCirPlan = QAction(QIcon('exit.png'), 'Open circular1 plan', self)
        openCirPlan.setShortcut('Ctrl+c')
        openCirPlan.setStatusTip('Open circular1 plan form')
        openCirPlan.triggered.connect(self.gotoCircular1)
        planMenu.addAction(openCirPlan)

        #规划菜单栏:打直线规划规划
        openLinePlan = QAction(QIcon('exit.png'), 'Open Line1 plan', self)
        openLinePlan.setShortcut('Ctrl+l')
        openLinePlan.setStatusTip('Open Line1 plan form')
        openLinePlan.triggered.connect(self.gotoLine1)
        planMenu.addAction(openLinePlan)

        # -------------------积分自适应导纳菜单-------------------#
        #阻抗控制菜单栏:阻抗参数调试
        openImp1 = QAction(QIcon('exit.png'), 'Impedance paramter debug ', self)
        openImp1.setStatusTip('Open impedance form1')
        openImp1.triggered.connect(self.gotoImp1)
        impedanceMenu.addAction(openImp1)

        #阻抗控制菜单栏:阻抗轨迹控制
        openImp2 = QAction(QIcon('exit.png'), 'Impedance controll', self)
        openImp2.setStatusTip('Open impedance form2')
        openImp2.triggered.connect(self.gotoImp2)
        impedanceMenu.addAction(openImp2)

        #阻抗控制菜单栏:等效刚度评估
        openImp3 = QAction(QIcon('exit.png'), 'Equivalent stiffness evaluation', self)
        openImp3.setStatusTip('Open impedance form3')
        openImp3.triggered.connect(self.gotoImp3)
        impedanceMenu.addAction(openImp3)

        # ----------------------示教菜单栏------------------------#
        #示教菜单栏:
        openteach1 = QAction(QIcon('exit.png'), 'Open openTeachn1', self)
        openteach1.setStatusTip('Open teach form1')
        openteach1.triggered.connect(self.gototeach1)
        teachMenu.addAction(openteach1)

        #----------------------工艺菜单栏------------------------#
        # 运行给定数据
        openTechn1 = QAction(QIcon('exit.png'), 'Open polish', self)
        openTechn1.setStatusTip('Open technology polish form')
        openTechn1.triggered.connect(self.gotoTechnology)
        demoMenu.addAction(openTechn1)

        #运行给定数据
        openData = QAction(QIcon('exit.png'), 'Open run data', self)
        openData.setStatusTip('Open run data form')
        openData.triggered.connect(self.gotoData)
        demoMenu.addAction(openData)

        # ----------------------测试菜单栏------------------------#
        #测试菜单栏
        openTest1 = QAction(QIcon('exit.png'), 'Open fitt test1', self)
        openTest1.setStatusTip('Open run test form1')
        openTest1.triggered.connect(self.gotoTest1)
        testMenu.addAction(openTest1)

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_wish_pos.clicked.connect(self.read_wish_pos)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_plan.clicked.connect(self.plan)
        self.checkBox_gazebo.stateChanged.connect(self.gazebo_or_real)

    #打开文件的地址和内容
    def fileOpen(self):
        #打开文件操作
        fname = QFileDialog.getOpenFileName(self, 'Open file', '/home')

        if fname[0]:
            f = open(fname[0], 'r')
            self.filedir = fname[0]

            with f:
                data = f.read()
                self.pubdata = data
                self.textEdit.setText(data)

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
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)
        # 绘制速度图
        self.p2.plot(t2, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t2, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t2, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t2, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t2, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t2, f[:, 5], pen='y', name='F6', clear=False)

    #切换到实物
    def gazebo_or_real(self):
        self.real_flag = self.checkBox_gazebo.isChecked()
        if(self.real_flag):
            self.sub_force_path = "/ft_sensor_topic"
            self.lineEdit_sub_f.setText(self.sub_force_path)
            self.sub_pos_path = "/joint_states"
            self.lineEdit_sub_qq.setText(self.sub_pos_path)
            self.pub_path = "/all_joints_position_group_controller/command"
            self.lineEdit_pub_qq.setText(self.pub_path)
        else:
            self.sub_force_path = "/ft_sensor_topic"
            self.lineEdit_sub_f.setText(self.sub_force_path)
            self.sub_pos_path = "/armc/joint_states"
            self.lineEdit_sub_qq.setText(self.sub_pos_path)
            self.pub_path = "/armc/joint_positions_controller/command"
            self.lineEdit_pub_qq.setText(self.pub_path)

    #读取给定关节角度
    def read_wish_pos(self):
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()
        msg_pos = "规划目标点\n" + \
                   "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) +\
                   "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) +\
                   "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                   "\n" + "q7:" + str(qq[6])
        self.textEdit.setText(msg_pos)
        self.qq_go = np.copy(qq*np.pi/180.0)

    #规划函数
    def plan(self):
        #获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        #调用规划函数
        [qq,qv,T] = gf.q_joint_space_plan(qq_b, self.qq_go)
        #调用绘图函数
        k =len(qq)
        t = np.linspace(0, T*(k-1), k)
        self.T = T
        #绘制关节角位置速度图
        self.plot_joint(t, qq, t, np.zeros([k, 6]))
        #将规划好的位置定义为全局变量
        self.command_qq_list = np.copy(qq)

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        self.state_t = self.state_t + self.T
        self.state_qq_list.append(qq)
        self.state_t_list.append(self.state_t)
        # 仅记录100个数据点
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
        self.state_f_list.append(f)
        # 仅记录1000个数据点
        del self.state_f_list[0]

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
        plot_t = np.array(self.state_t_list)
        plot_qq = np.array(self.state_qq_list)
        plot_f = np.array(self.state_f_list)
        self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

    def run_topic(self):
        #先读取地址
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

    def begin_function(self):
        #运行标签启动
        self.run_flag = True

        #求取数据长度
        kk = len(self.command_qq_list)

        if(not self.real_flag):
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
            if(not self.run_flag or k==kk):
                if(not self.real_flag):
                    self.timer_p.stop()
                    self.timer_plot.stop()
                break
            #发送数据
            command_data = Float64MultiArray()
            if (k < kk):
                command_data.data = self.command_qq_list[k, 0:self.n]
            self.pub.publish(command_data)
            if(k%20==0):
                pub_msg = "armc" + "第" + str(k) + "次" + "publisher data is: " + '\n'\
                      "q1:" + str(command_data.data[0]) + '\n' \
                        "q2:" + str(command_data.data[1]) + '\n' \
                          "q3:" + str(command_data.data[2]) + '\n'\
                            "q4:" + str(command_data.data[3]) + '\n' \
                               "q5:" + str(command_data.data[4]) + '\n' \
                                 "q6:" + str(command_data.data[5]) + '\n' \
                                    "q7:" + str(command_data.data[6]) + '\n'
                self.textEdit.setText(pub_msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    # ===============窗口跳转函数================#
    def gotoCircular1(self):
        self.hide()
        self.sub_cir1 = CirPlanWindow1()
        self.sub_cir1.show()

    def gotoLine1(self):
        self.hide()
        self.sub_line1 = LinePlanWindow1()
        self.sub_line1.show()

    def gotoImp1(self):
        self.hide()
        self.sub_imp1 = ImpWindow1()
        self.sub_imp1.show()

    def gotoImp2(self):
        self.hide()
        self.sub_imp2 = ImpWindow2()
        self.sub_imp2.show()

    def gotoImp3(self):
        self.hide()
        self.sub_imp3 = ImpWindow3()
        self.sub_imp3.show()

    def gotoData(self):
        self.hide()
        self.sub_data = RunDataWindow()
        self.sub_data.show()

    def gotoTest1(self):
        self.hide()
        self.test1 = TestWindow1()
        self.test1.show()

    def gototeach1(self):
        self.hide()
        self.teach1 =TeachWindow1()
        self.teach1.show()

    def gotoUrs1(self):
        self.hide()
        self.urs1 = UrsWindow1()
        self.urs1.show()

    def gotoUrs2(self):
        self.hide()
        self.urs2 = UrsWindow2()
        self.urs2.show()

    def gotoUrs3(self):
        self.hide()
        self.urs3 = UrsWindow3()
        self.urs3.show()

    def gotoArmct1(self):
        self.hide()
        self.armct1 = ArmctWindow1()
        self.armct1.show()

    def gotoArmct2(self):
        self.hide()
        self.armct2 = ArmctWindow2()
        self.armct2.show()

    def gotoArmct3(self):
        self.hide()
        self.armct3 = ArmctWindow3()
        self.armct3.show()

    def gotoTechnology(self):
        self.hide()
        self.Technology1 = TechnologyWindow1()
        self.Technology1.show()

    def gotoHand1(self):
        self.hide()
        self.hand1 = UrsHandWindow1()
        self.hand1.show()

    def gotoArmc1(self):
        self.hide()
        self.armc1 = ArmcWindow1()
        self.armc1.show()

    def gotoArmc2(self):
        self.hide()
        self.armc2 = ArmcWindow2()
        self.armc2.show()

#***************************armc窗口：加入速度控制********************************#
class ArmcWindow1(QMainWindow, Ui_ArmcForm1):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0

    def __init__(self, parent=None):
        super(ArmcWindow1, self).__init__(parent)
        self.T = 0.01
        self.run_flag = True  # 开始或停止标签
        self.vel_flag = False
        self.real_flag = False
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
        self.button_wish_pos.clicked.connect(self.read_wish_pos)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_plan.clicked.connect(self.plan)
        self.checkBox_gazebo.stateChanged.connect(self.vel_or_real)
        self.checkBox_vel.stateChanged.connect(self.vel_or_real)

    # 打开文件的地址和内容
    def fileOpen(self):
        # 打开文件操作
        fname = QFileDialog.getOpenFileName(self, 'Open file', '/home')

        if fname[0]:
            f = open(fname[0], 'r')
            self.filedir = fname[0]

            with f:
                data = f.read()
                self.pubdata = data
                self.textEdit.setText(data)

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

        p2 = win2.addPlot(title="joint vel")  # 添加第一个绘图窗口
        p2.setLabel('left', text='vel/(rad/s)', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1, p2

    # 绘画关节角和关节角速度曲线
    def plot_pos(self, t1, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)

    def plot_vel(self, t1, qv):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p2.plot(t1, qv[:, 0], pen='b', name='qv1', clear=True)
        self.p2.plot(t1, qv[:, 1], pen='g', name='qv2', clear=False)
        self.p2.plot(t1, qv[:, 2], pen='r', name='qv3', clear=False)
        self.p2.plot(t1, qv[:, 3], pen='c', name='qv4', clear=False)
        self.p2.plot(t1, qv[:, 4], pen='m', name='qv5', clear=False)
        self.p2.plot(t1, qv[:, 5], pen='y', name='qv6', clear=False)
        self.p2.plot(t1, qv[:, 6], pen='w', name='qv7', clear=False)

    # 切换到实物
    def vel_or_real(self):
        self.real_flag = self.checkBox_gazebo.isChecked()
        self.vel_flag = self.checkBox_vel.isChecked()
        if (self.real_flag):
            if(self.vel_flag):
                self.sub_force_path = "/armc/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/all_joints_velocity_group_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)
            else:
                self.sub_force_path = "/armc/ft_sensor_topic"
                self.lineEdit_sub_f.setText(self.sub_force_path)
                self.sub_pos_path = "/joint_states"
                self.lineEdit_sub_qq.setText(self.sub_pos_path)
                self.pub_path = "/all_joints_position_group_controller/command"
                self.lineEdit_pub_qq.setText(self.pub_path)
        else:
            self.sub_force_path = "/ft_sensor_topic"
            self.lineEdit_sub_f.setText(self.sub_force_path)
            self.sub_pos_path = "/armc/joint_states"
            self.lineEdit_sub_qq.setText(self.sub_pos_path)
            self.pub_path = "/armc/joint_positions_controller/command"
            self.lineEdit_pub_qq.setText(self.pub_path)

    # 读取给定关节角度
    def read_wish_pos(self):
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()
        msg_pos = "规划目标点\n" + \
                  "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                  "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                  "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                  "\n" + "q7:" + str(qq[6])
        self.textEdit.setText(msg_pos)
        self.qq_go = np.copy(qq * np.pi / 180.0)

    # 规划函数
    def plan(self):
        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        # 调用规划函数
        [qq, qv, T] = gf.q_joint_space_plan(qq_b, self.qq_go)
        # 调用绘图函数
        k = len(qq)
        t = np.linspace(0, T * (k - 1), k)
        self.T = T
        # 绘制关节角位置图
        self.plot_pos(t, qq)
        # 绘制关节角速度图
        self.plot_vel(t, qv)

        # 将规划好的位置定义为全局变量
        self.command_qq_list = np.copy(qq)
        self.command_qv_list = np.copy(qv)

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        self.state_t = self.state_t + self.T
        self.state_qq_list.append(qq)
        self.state_t_list.append(self.state_t)
        # 仅记录100个数据点
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
        #plot_f = np.array(self.state_f_list)
        self.plot_pos(plot_t, plot_qq)

    def run_topic(self):
        # 先读取地址
        self.sub_force_path = str(self.lineEdit_sub_f.text())
        self.sub_pos_path = str(self.lineEdit_sub_qq.text())
        self.pub_path = str(self.lineEdit_pub_qq.text())
        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos_path, JointState, self.joint_callback)
        #rospy.Subscriber(self.sub_force_path, WrenchStamped, self.force_callback)
        self.pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"
        self.textEdit.setText(msg_tip)
        t1.start()

    def begin_function(self):
        if(self.vel_flag):
            self.run_vel()
        else:
            self.run_pos()

    def run_pos(self):
        # 运行标签启动
        self.run_flag = True

        # 求取数据长度
        kk = len(self.command_qq_list)

        if (not self.real_flag):
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
                if (not self.real_flag):
                    self.timer_p.stop()
                    self.timer_plot.stop()
                break
            # 发送数据
            command_data = Float64MultiArray()
            if (k < kk):
                command_data.data = self.command_qq_list[k, 0:self.n]
            else:
                break
                command_data.data = self.command_qq_list[-1, 0:self.n]
            self.pub.publish(command_data)
            if (k/10 == 0):
                pub_msg = "armc" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                "q1:" + str(
                    command_data.data[0]) + '\n' \
                                            "q2:" + str(command_data.data[1]) + '\n' \
                                                                                "q3:" + str(
                    command_data.data[2]) + '\n' \
                                            "q4:" + str(command_data.data[3]) + '\n' \
                                                                                "q5:" + str(
                    command_data.data[4]) + '\n' \
                                            "q6:" + str(command_data.data[5]) + '\n' \
                                                                                "q7:" + str(
                    command_data.data[6]) + '\n'
                self.textEdit.setText(pub_msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    def run_vel(self):
        # 运行标签启动
        self.run_flag = True

        # 求取数据长度
        kk = len(self.command_qv_list)

        if (not self.real_flag):
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
                if (not self.real_flag):
                    self.timer_p.stop()
                    self.timer_plot.stop()
                break
            # 发送数据
            command_data = Float64MultiArray()
            if (k < kk):
                command_data.data = self.command_qv_list[k, 0:self.n]
            else:
                break
                command_data.data = np.zeros(self.n)
            self.pub.publish(command_data)
            if (k%50 == 0):
                pub_msg = "armc" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                "qv1:" + str(
                    command_data.data[0]) + '\n' \
                                            "qv2:" + str(command_data.data[1]) + '\n' \
                                                                                "qv3:" + str(
                    command_data.data[2]) + '\n' \
                                            "qv4:" + str(command_data.data[3]) + '\n' \
                                                                                "qv5:" + str(
                    command_data.data[4]) + '\n' \
                                            "qv6:" + str(command_data.data[5]) + '\n' \
                                                                                "qv7:" + str(
                    command_data.data[6]) + '\n'
                self.textEdit.setText(pub_msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    # ===============窗口跳转函数================#
    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================阻抗控制窗口1:阻抗参数调试================#
class ArmcWindow2(QMainWindow, Ui_ArmcForm2):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0

    def __init__(self, parent=None):
        super(ArmcWindow2, self).__init__(parent)
        self.T = 0.01
        self.t = 15

        self.run_flag = False  # 开始或停止标签
        self.run_finish = True
        self.real_flag = False
        self.joint_flag = False

        self.init_flag = False
        self.inc_flag = False
        self.home_flag = False

        self.pos1 = False
        self.pos2 = False

        self.read_flag = False

        [DH0, q_max, q_min] = gf.get_robot_parameter("armc")
        self.DH0 = DH0
        self.q_max = q_max
        self.q_min = q_min
        self.robot = "armc"

        #建立规划类
        self.line_plan = pap.ArmcLinePlan()
        self.line_plan.get_period(self.T)
        self.line_plan.get_robot_parameter(DH0, q_min, q_max)

        self.sub_pos_path = "/armc/joint_states"
        self.pub_path = "/armc/joint_positions_controller/command"
        self.n = 7  #机械臂关节数

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
        self.button_read.clicked.connect(self.read_qq)
        self.checkBox_real.stateChanged.connect(self.real_and_joint)
        self.button_get_init_xx.clicked.connect(self.calculation_init_point_xx)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_init.clicked.connect(self.go_init)
        self.button_inc.clicked.connect(self.go_inc)
        self.button_home.clicked.connect(self.go_home)

        #-------增量控制按钮栏--------#
        self.button_qq1_N.clicked.connect(self.fun_qq1_n)
        self.button_qq2_N.clicked.connect(self.fun_qq2_n)
        self.button_qq3_N.clicked.connect(self.fun_qq3_n)
        self.button_qq4_N.clicked.connect(self.fun_qq4_n)
        self.button_qq5_N.clicked.connect(self.fun_qq5_n)
        self.button_qq6_N.clicked.connect(self.fun_qq6_n)
        self.button_qq7_N.clicked.connect(self.fun_qq7_n)
        self.button_qq1_P.clicked.connect(self.fun_qq1_p)
        self.button_qq2_P.clicked.connect(self.fun_qq2_p)
        self.button_qq3_P.clicked.connect(self.fun_qq3_p)
        self.button_qq4_P.clicked.connect(self.fun_qq4_p)
        self.button_qq5_P.clicked.connect(self.fun_qq5_p)
        self.button_qq6_P.clicked.connect(self.fun_qq6_p)
        self.button_qq7_P.clicked.connect(self.fun_qq7_p)
        self.button_xx1_N.clicked.connect(self.fun_xx1_n)
        self.button_xx2_N.clicked.connect(self.fun_xx2_n)
        self.button_xx3_N.clicked.connect(self.fun_xx3_n)
        self.button_xx4_N.clicked.connect(self.fun_xx4_n)
        self.button_xx5_N.clicked.connect(self.fun_xx5_n)
        self.button_xx6_N.clicked.connect(self.fun_xx6_n)
        self.button_xx1_P.clicked.connect(self.fun_xx1_p)
        self.button_xx2_P.clicked.connect(self.fun_xx2_p)
        self.button_xx3_P.clicked.connect(self.fun_xx3_p)
        self.button_xx4_P.clicked.connect(self.fun_xx4_p)
        self.button_xx5_P.clicked.connect(self.fun_xx5_p)
        self.button_xx6_P.clicked.connect(self.fun_xx6_p)


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

        p2 = win2.addPlot(title="joint vel")  # 添加第一个绘图窗口
        p2.setLabel('left', text='vel/N', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1, p2

        # 绘画关节角和关节角速度曲线

    def plot_pos(self, t1, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)

    def plot_vel(self, t1, qv):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p2.plot(t1, qv[:, 0], pen='b', name='qv1', clear=True)
        self.p2.plot(t1, qv[:, 1], pen='g', name='qv2', clear=False)
        self.p2.plot(t1, qv[:, 2], pen='r', name='qv3', clear=False)
        self.p2.plot(t1, qv[:, 3], pen='c', name='qv4', clear=False)
        self.p2.plot(t1, qv[:, 4], pen='m', name='qv5', clear=False)
        self.p2.plot(t1, qv[:, 5], pen='y', name='qv6', clear=False)
        self.p2.plot(t1, qv[:, 6], pen='w', name='qv7', clear=False)

    # 支持armt、armc四种状态切换
    def real_and_joint(self):
        self.real_flag = self.checkBox_real.isChecked()
        if (self.real_flag):
            self.sub_pos_path = "/joint_states"
            self.pub_path = "/all_joints_position_group_controller/command"

        else:
            self.sub_pos_path = "/armc/joint_states"
            self.pub_path = "/armc/joint_positions_controller/command"

    # 计算初始位置
    def calculation_init_point_xx(self):
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
        xx[3:] = xx_b[3:] * 180.0/np.pi  # 转换为mm显示

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

    def read_qq(self):
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
        qq1 = qq * np.pi / 180.0
        self.qq_init = np.copy(qq1)

        msg = "读取目标关节角:\n" + \
                    "q1:"  + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + "\n" + \
                    "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + "\n" + \
                    "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + "\n" + \
                    "q7:" + str(qq[6]) + "\n"
        self.textEdit.setText(msg)

        self.read_flag = True

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

        msg_joint = "初始关节角\n" + \
                    "qq:" + "[" + str(qq[0]) + "," + str(qq[1]) + "," + \
                    str(qq[2]) + "," + str(qq[3]) + "," + str(qq[4]) + \
                    "," + str(qq[5]) + "," + str(qq[6]) + "]" + "\n"

        msg_pos = "期望末端位置\n" + \
                  "Xd:" + "[" + str(xx[0]) + "," + str(xx[1]) + "," + \
                  str(xx[2]) + "," + str(xx[3]) + "," + str(xx[4]) + \
                  "," + str(xx[5]) + "]" + "\n"


        msg = msg_joint + msg_pos
        self.textEdit.setText(msg)
        # 转化为输入单位，并保存到全局变量
        self.qq_init = np.copy(qq * np.pi / 180.0)
        self.xx_d = np.copy(x_d)
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
        self.plot_pos(plot_t, plot_qq)

    def run_topic(self):
        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos_path, JointState, self.joint_callback)
        self.pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=100)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"
        self.textEdit.setText(msg_tip)
        t1.start()

    def go_init(self):
        if (not self.read_flag):
            msg = "未读取参数！\n"
            self.textEdit.setText(msg)
            return -1

        self.init_flag = True
        self.inc_flag = False
        self.home_flag = False

        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        # 调用规划函数
        [qq, qv, _] = gf.q_joint_space_plan_time(qq_b, self.qq_init, self.T, self.t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq_init = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    def go_inc(self):
        self.init_flag = False
        self.inc_flag = True
        self.home_flag = False

        #创建运动学求解器
        self.my_kin = kin.GeneralKinematic(self.DH0, self.q_min, self.q_max)

        xx = self.my_kin.fkine_euler(self.qq_state)
        self.XX = np.copy(xx)
        self.qq = np.copy(self.qq_state)
        qq = self.qq_state*180/np.pi
        qq = np.around(qq, 2)
        xx[:3] = xx[:3]*1000
        xx[3:] = xx[3:]*180/np.pi
        xx = np.around(xx, 2)

        self.lineEdit_qq1.setText(str(qq[0]))
        self.lineEdit_qq2.setText(str(qq[1]))
        self.lineEdit_qq3.setText(str(qq[2]))
        self.lineEdit_qq4.setText(str(qq[3]))
        self.lineEdit_qq5.setText(str(qq[4]))
        self.lineEdit_qq6.setText(str(qq[5]))
        self.lineEdit_qq7.setText(str(qq[6]))

        self.lineEdit_xx1.setText(str(xx[0]))
        self.lineEdit_xx2.setText(str(xx[1]))
        self.lineEdit_xx3.setText(str(xx[2]))
        self.lineEdit_xx4.setText(str(xx[3]))
        self.lineEdit_xx5.setText(str(xx[4]))
        self.lineEdit_xx6.setText(str(xx[5]))

        msg = "已切换到增量模式！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        self.init_flag = False
        self.inc_flag = False
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
        self.plot_vel(t, qv)
        self.plot_pos(t, qq)
        # 将规划好的位置定义为全局变量
        self.command_qq_home = np.copy(qq)
        msg = "运动到家点已规划！\n"
        self.textEdit.setText(msg)

    #增量运动程序
    def inc_run(self):
        t = 5
        qq_command = np.zeros(self.n)
        xx2 = np.zeros(6)
        if(self.joint_flag):
            qq_command[0] = self.lineEdit_qq1.text()
            qq_command[1] = self.lineEdit_qq2.text()
            qq_command[2] = self.lineEdit_qq3.text()
            qq_command[3] = self.lineEdit_qq4.text()
            qq_command[4] = self.lineEdit_qq5.text()
            qq_command[5] = self.lineEdit_qq6.text()
            qq_command[6] = self.lineEdit_qq7.text()
            qq_command = qq_command*np.pi/180.0

            #判断是否超出关节极限
            flag = gf.out_joint_limit(qq_command, self.q_min, self.q_max)
            if(flag):
                msg = "超出关节极限！！！\n" + "请反向操作撤回！"
                self.textEdit.setText(msg)
                return -1

            t = int(self.dq)/5 + 5
            #更新末端位置
            xx = self.my_kin.fkine_euler(qq_command)
            xx[:3] = xx[:3] * 1000
            xx[3:] = xx[3:] * 180.0 / np.pi
            xx = np.around(xx, 2)
            self.lineEdit_xx1.setText(str(xx[0]))
            self.lineEdit_xx2.setText(str(xx[1]))
            self.lineEdit_xx3.setText(str(xx[2]))
            self.lineEdit_xx4.setText(str(xx[3]))
            self.lineEdit_xx5.setText(str(xx[4]))
            self.lineEdit_xx6.setText(str(xx[5]))
            [qq_list, qv, qa] = gf.q_joint_space_plan_time(self.qq_state, qq_command,
                                                           T=self.T, t=t)

        else:
            xx2[0] = self.lineEdit_xx1.text()
            xx2[1] = self.lineEdit_xx2.text()
            xx2[2] = self.lineEdit_xx3.text()
            xx2[3] = self.lineEdit_xx4.text()
            xx2[4] = self.lineEdit_xx5.text()
            xx2[5] = self.lineEdit_xx6.text()
            xx2[:3] = xx2[:3]*0.001
            xx2[3:] = xx2[3:]*np.pi/180.0
            t = int(self.dx)/5 + 5
            #设计笛卡尔空间规划
            xx1 = self.my_kin.fkine_euler(self.qq_state)
            self.line_plan.get_begin_end_point(xx1, xx2)
            self.line_plan.get_init_guess_joint(self.qq_state)

            [qq_list, qv, qa] = self.line_plan.out_joint()

            #更新关节角度
            qq = qq_list[-1, :] * 180 / np.pi
            qq1 = np.around(qq, 1)
            self.lineEdit_qq1.setText(str(qq1[0]))
            self.lineEdit_qq2.setText(str(qq1[1]))
            self.lineEdit_qq3.setText(str(qq1[2]))
            self.lineEdit_qq4.setText(str(qq1[3]))
            self.lineEdit_qq5.setText(str(qq1[4]))
            self.lineEdit_qq6.setText(str(qq1[5]))
            self.lineEdit_qq7.setText(str(qq1[6]))

        # 绘制关节角位置速度图
        num = len(qq_list)
        t1 = np.linspace(0, self.T*(num - 1), num)
        self.plot_vel(t1, qv)
        self.plot_pos(t1, qq_list)
        self.command_qq_inc = np.copy(qq_list)
        self.begin_function()

    #---------增量按钮组---------#
    def fun_qq1_n(self):
        if(not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq1.text())
        qq = qq - self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq1.setText(str(qq))
        #启动程序
        self.inc_run()

    def fun_qq2_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq2.text())
        qq = qq - self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq2.setText(str(qq))
        #启动程序
        self.inc_run()

    def fun_qq3_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq3.text())
        qq = qq - self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq3.setText(str(qq))
        #启动程序
        self.inc_run()

    def fun_qq4_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq4.text())
        qq = qq - self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq4.setText(str(qq))
        #启动程序
        self.inc_run()

    def fun_qq5_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq5.text())
        qq = qq - self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq5.setText(str(qq))
        #启动程序
        self.inc_run()

    def fun_qq6_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq6.text())
        qq = qq - self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq6.setText(str(qq))
        #启动程序
        self.inc_run()

    def fun_qq7_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq7.text())
        qq = qq - self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq7.setText(str(qq))
        #启动程序
        self.inc_run()

    def fun_qq1_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq1.text())
        qq = qq + self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq1.setText(str(qq))
        # 启动程序
        self.inc_run()

    def fun_qq2_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq2.text())
        qq = qq + self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq2.setText(str(qq))
        # 启动程序
        self.inc_run()

    def fun_qq3_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq3.text())
        qq = qq + self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq3.setText(str(qq))
        # 启动程序
        self.inc_run()

    def fun_qq4_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq4.text())
        qq = qq + self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq4.setText(str(qq))
        # 启动程序
        self.inc_run()

    def fun_qq5_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq5.text())
        qq = qq + self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq5.setText(str(qq))
        # 启动程序
        self.inc_run()

    def fun_qq6_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq6.text())
        qq = qq + self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq6.setText(str(qq))
        # 启动程序
        self.inc_run()

    def fun_qq7_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq7.text())
        qq = qq + self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq7.setText(str(qq))
        # 启动程序
        self.inc_run()

    def fun_xx1_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx1.text())
        xx = xx - self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx1.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx2_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx2.text())
        xx = xx - self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx2.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx3_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx3.text())
        xx = xx - self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx3.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx4_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx4.text())
        xx = xx - self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx4.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx5_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx5.text())
        xx = xx - self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx5.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx6_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx6.text())
        xx = xx - self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx6.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx1_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx1.text())
        xx = xx + self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx1.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx2_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx2.text())
        xx = xx + self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx2.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx3_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx3.text())
        xx = xx + self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx3.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx4_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx4.text())
        xx = xx + self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx4.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx5_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx5.text())
        xx = xx + self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx5.setText(str(xx))
        # 启动程序
        self.inc_run()

    def fun_xx6_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx6.text())
        xx = xx + self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx6.setText(str(xx))
        # 启动程序
        self.inc_run()
    #----------增量按钮结束---------#

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True
        self.run_finish = False

        # 提示标语
        msg = "开始下发命令！\n"
        self.textEdit.setText(msg)

        # 设置绘图,用Qtimer开线程处理（线程4）
        if(not self.real_flag):
            self.timer_plot = QTimer()
            self.timer_plot.timeout.connect(self.realtime_plot)
            self.timer_plot.start(1000)

        # 发送关节角度
        rate = rospy.Rate(100)
        kk = 0
        k = 0
        if (self.init_flag):
            kk = len(self.command_qq_init)

        if (self.inc_flag):
            kk = len(self.command_qq_inc)

        if (self.home_flag):
            kk = len(self.command_qq_home)

        # # 进度条显示时间间隔
        show_time = int(kk * self.T * 10)

        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(show_time)

        while not rospy.is_shutdown():
            # 检测是否启动急停
            if ((not self.run_flag) or k==kk):
                #self.timer_p.stop()
                if(not self.real_flag):
                    self.timer_plot.stop()
                break

            command_data = Float64MultiArray()
            if (self.init_flag):
                command_data.data = self.command_qq_init[k, :]
            if (self.inc_flag):
                command_data.data = self.command_qq_inc[k, :]
            if (self.home_flag):
                command_data.data = self.command_qq_home[k, :]
            #仿真1关节不动
            self.pub.publish(command_data)
            if (k%10 == 0):
                msg = "armc" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                "q1:" + str(
                    command_data.data[0]) + '\n' \
                                            "q2:" + str(command_data.data[1]) + '\n' \
                                                                                "q3:" + str(
                    command_data.data[2]) + '\n' \
                                            "q4:" + str(command_data.data[3]) + '\n' \
                                                                                "q5:" + str(
                    command_data.data[4]) + '\n' \
                                            "q6:" + str(command_data.data[5]) + '\n' \
                                                                                "q7:" + str(
                    command_data.data[6]) + '\n'
                self.textEdit.setText(msg)
            k = k + 1
            QApplication.processEvents()
            rate.sleep()
        msg = "运动完成！"
        if (not self.real_flag):
            self.timer_plot.stop()
        self.textEdit.setText(msg)
        self.run_finish = True

    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

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
        command_qq1 = 0
        command_qq2 = 0

        if(self.robot1_flag or self.all_flag):
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
        self.timer_p.start(10*self.T*kk)

        rate = rospy.Rate(100)
        msg = ''
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
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

# ================圆规划窗口1================#
class CirPlanWindow1(QMainWindow, Ui_CirForm1):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_xx_list = list(np.zeros([1000, 3]))
    state_t_list = list(np.zeros(1000))
    state_t = 0
    flag = 1  # 开始或停止标签
    arm_flag = False
    gazebo_flag = 0
    sub_path = "/robot3/joint_states"
    pub_path = "/robot3/armc_position_controller/command"
    n = 7  # 机械臂关节数
    def __init__(self, parent=None):
        super(CirPlanWindow1, self).__init__(parent)
        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')

        #文件菜单:返回主窗口
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
        self.button_end.clicked.connect(self.off)
        self.button_plan.clicked.connect(self.plan)
        self.checkBox_UR5.stateChanged.connect(self.armc_or_ur5)
        self.button_plan.clicked.connect(self.plan)
        self.button_begin_point.clicked.connect(self.read_init_parameter)
        self.button_begin_end_point.clicked.connect(self.get_end_begin_point)
        self.button_plan_2.clicked.connect(self.read_init_point)

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

        p2 = win2.addPlot(title="TCP")  # 添加第一个绘图窗口
        p2.setLabel('left', text='vel:mm/s', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1, p2

    # 绘画关节角和关节角速度曲线
    def plot_joint(self, t1, qq, t2, xx):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)
        # 绘制操作空间位置图
        self.p2.plot(t2, xx[:, 0], pen='b', name='x1', clear=True)
        self.p2.plot(t2, xx[:, 1], pen='g', name='x2', clear=False)
        self.p2.plot(t2, xx[:, 2], pen='r', name='x3', clear=False)

    # 计算初始位置
    def read_init_point(self):
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
        xx_b = gf.get_begin_point(qq, self.arm_flag)

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
        self.xx_b = np.copy(xx_b)

    # 读取给定关节角度
    def read_init_parameter(self):
        #初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()

        #规划起点，采用欧拉zyx或RPY角
        xx = np.zeros(6)
        xx[0] = self.lineEdit_x1.text()
        xx[1] = self.lineEdit_x2.text()
        xx[2] = self.lineEdit_x3.text()
        xx[3] = self.lineEdit_x4.text()
        xx[4] = self.lineEdit_x5.text()
        xx[5] = self.lineEdit_x6.text()
        xx_b = np.copy(xx)
        xx_b[0:3] = xx[0:3]/1000.0

        #规划时间和周期
        self.T = float(self.lineEdit_T.text())
        self.t = float(self.lineEdit_t.text())

        #获取规划半径
        rc = float(self.lineEdit_r.text())

        msg_joint = "初始关节角\n" + \
                  "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                  "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                  "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                  "\n" + "q7:" + str(qq[6]) + "\n"
        msg_pos = "初始末端位置\n" + \
                  "x1:" + str(xx[0]) + "\n" + "x2:" + str(xx[1]) + \
                  "\n" + "x3:" + str(xx[2]) + "\n" + "x4:" + str(xx[3]) + \
                  "\n" + "x5:" + str(xx[4]) + "\n" + "x6:" + str(xx[5]) + "\n"

        msg_time = "规划时间和周期\n" + \
                  "T:" + str(self.T) + "\n" + "t:" + str(self.t) + "\n"

        msg_cir = "规划半径\n" + "rc:" + str(rc)
        msg = msg_joint + msg_pos + msg_time + msg_cir
        self.textEdit.setText(msg)
        #转化为m\s单位
        self.qq_b = np.copy(qq*np.pi/180.0)
        self.xx_b = np.copy(xx_b)
        self.rc = rc/1000.0

    def get_end_begin_point(self):
        xx_b = gf.get_begin_point(self.qq_b,self.arm_flag)
        xx = np.copy(xx_b) #转化为mm显示
        xx[0:3] = xx_b[0:3]*1000 #转换为mm显示
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
        self.xx_b = np.copy(xx_b)

    def plan(self):
        qq_b = self.qq_b
        if (self.arm_flag == True):
            qq_b = self.qq_b[0:6]
        # 调用规划函数
        [xe,qq] = gf.cir_plan(qq_b, self.xx_b,
                              self.rc, self.T, self.t,
                              self.arm_flag)
        # 调用绘图函数
        k1 = len(qq[:, 0])
        tt1 = np.linspace(0, self.T * (k1 - 1), k1)
        k2 = len(xe[:, 0])
        tt2 = np.linspace(0, self.T * (k2 - 1), k2)
        # 绘制关节角位置速度图
        self.plot_joint(tt1, qq, tt2, xe)
        # 将规划好的位置定义为全局变量
        self.qq_list = np.copy(qq)

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        # 正运动计算
        xx = gf.get_begin_point(qq, self.flag)

        # 存储数据
        self.state_t = self.state_t + self.T
        self.state_qq_list.append(qq)
        self.state_t_list.append(self.state_t)
        self.state_xx_list.append(xx[0:3])
        # 仅记录1000个数据点
        del self.state_t_list[0]
        del self.state_qq_list[0]
        del self.state_xx_list[0]

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def probar_show(self):
        self.step_p = self.step_p + 1
        self.progressBar.setValue(self.step_p)
        if (self.step_p > 99):
            self.timer_p.stop()

    def off(self):
        self.flag = 0

    def realtime_plot(self):
        plot_t = np.array(self.state_t_list)
        plot_qq = np.array(self.state_qq_list)
        plot_xx = np.array(self.state_xx_list)
        self.plot_joint(plot_t, plot_qq, plot_t, plot_xx)

    def armc_or_ur5(self):
        # 运行该函数切换到ur5
        self.sub_path = "/robot1/joint_states"
        self.pub_path = "/robot1/ur5_position_controller/command"
        self.arm_flag = True
        self.n = 6

    def begin_function(self):
        # 运行标签启动
        self.flag = 1

        # 运行话题
        rospy.init_node('mainWindon_run!')
        rospy.Subscriber(self.sub_path, JointState, self.joint_callback)
        pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=10)

        # 求取数据长度
        kk = len(self.qq_list)

        # 进度条显示时间间隔
        show_time = int(kk * self.T * 10)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg1 = "MainWindon run!;"
        self.textEdit.setText(msg1)
        t1.start()

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
            if (self.flag == 0):
                self.timer_plot.stop()
                break
            # 发送数据
            command_data = Float64MultiArray()
            if (k < kk):
                command_data.data = self.qq_list[k, 0:self.n]
            else:
                command_data.data = self.qq_list[-1, 0:self.n]
            pub.publish(command_data)
            if (k % 10 == 0):
                if (self.n == 7):
                    pub_msg = "armc" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                    "q1:" + str(
                        command_data.data[0]) + '\n' \
                                                "q2:" + str(command_data.data[1]) + '\n' \
                                                                                    "q3:" + str(
                        command_data.data[2]) + '\n' \
                                                "q4:" + str(command_data.data[3]) + '\n' \
                                                                                    "q5:" + str(
                        command_data.data[4]) + '\n' \
                                                "q6:" + str(command_data.data[5]) + '\n' \
                                                                                    "q7:" + str(
                        command_data.data[6]) + '\n'
                else:
                    pub_msg = "UR5" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                   "q1:" + str(
                        command_data.data[0]) + '\n' \
                                                "q2:" + str(command_data.data[1]) + '\n' \
                                                                                    "q3:" + str(
                        command_data.data[2]) + '\n' \
                                                "q4:" + str(command_data.data[3]) + '\n' \
                                                                                    "q5:" + str(
                        command_data.data[4]) + '\n' \
                                                "q6:" + str(command_data.data[5]) + '\n'
                self.textEdit.setText(pub_msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================直线规划窗口1================#
class LinePlanWindow1(QMainWindow, Ui_LineForm1):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_xx_list = list(np.zeros([1000, 3]))
    state_t_list = list(np.zeros(1000))
    state_t = 0
    flag = 1  # 开始或停止标签
    arm_flag = False
    gazebo_flag = 0
    sub_path = "/robot3/joint_states"
    pub_path = "/robot3/armc_position_controller/command"
    n = 7  # 机械臂关节数

    def __init__(self, parent=None):
        super(LinePlanWindow1, self).__init__(parent)
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
        self.button_end.clicked.connect(self.off)
        self.button_plan.clicked.connect(self.plan)
        self.checkBox_UR5.stateChanged.connect(self.armc_or_ur5)
        self.button_begin_point.clicked.connect(self.read_begin_point)
        self.button_end_point.clicked.connect(self.read_end_point)
        self.button_plan__begin.clicked.connect(self.read_init_point)

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

        p2 = win2.addPlot(title="TCP")  # 添加第一个绘图窗口
        p2.setLabel('left', text='vel:mm/s', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1, p2

    # 绘画关节角和关节角速度曲线
    def plot_joint(self, t1, qq, t2, xx):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)
        # 绘制操作空间位置图
        self.p2.plot(t2, xx[:, 0], pen='b', name='x1', clear=True)
        self.p2.plot(t2, xx[:, 1], pen='g', name='x2', clear=False)
        self.p2.plot(t2, xx[:, 2], pen='r', name='x3', clear=False)

    # 计算初始位置
    def read_init_point(self):
        # 初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()

        #转换单位
        qq = qq*np.pi/180.0

        #计算初始位置
        xx_b = gf.get_begin_point(qq, self.arm_flag)

        #转换到显示单位
        xx = np.copy(xx_b)  # 转化为mm显示
        xx[0:3] = xx_b[0:3] * 1000  # 转换为mm显示

        #显示到界面
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
        self.xx_b = np.copy(xx_b)

    # 读取所有初始参数
    def read_begin_point(self):
        # 初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()

        # 规划起点，采用欧拉zyx或RPY角
        xx = np.zeros(6)
        xx[0] = self.lineEdit_x1.text()
        xx[1] = self.lineEdit_x2.text()
        xx[2] = self.lineEdit_x3.text()
        xx[3] = self.lineEdit_x4.text()
        xx[4] = self.lineEdit_x5.text()
        xx[5] = self.lineEdit_x6.text()
        x_b = np.copy(xx)
        x_b[0:3] = xx[0:3] / 1000.0

        # 规划时间和周期
        self.T = float(self.lineEdit_T.text())
        self.t = float(self.lineEdit_t.text())

        msg_joint = "初始关节角\n" + \
                    "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                    "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                    "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                    "\n" + "q7:" + str(qq[6]) + "\n"
        msg_pos = "直线起点\n" + \
                  "x1:" + str(xx[0]) + "\n" + "x2:" + str(xx[1]) + \
                  "\n" + "x3:" + str(xx[2]) + "\n" + "x4:" + str(xx[3]) + \
                  "\n" + "x5:" + str(xx[4]) + "\n" + "x6:" + str(xx[5]) + "\n"

        msg_time = "规划时间和周期\n" + \
                   "T:" + str(self.T) + "\n" + "t:" + str(self.t) + "\n"

        msg = msg_joint + msg_pos + msg_time
        self.textEdit.setText(msg)
        # 转化为输入单位，并保存到全局变量
        self.qq_b = np.copy(qq * np.pi / 180.0)
        self.xx_b = np.copy(x_b)

    def read_end_point(self):
        # 规划起点，采用欧拉zyx或RPY角
        xx = np.zeros(6)
        xx[0] = self.lineEdit_x1.text()
        xx[1] = self.lineEdit_x2.text()
        xx[2] = self.lineEdit_x3.text()
        xx[3] = self.lineEdit_x4.text()
        xx[4] = self.lineEdit_x5.text()
        xx[5] = self.lineEdit_x6.text()
        #转换为输入单位
        x_e = np.copy(xx)
        x_e[0:3] = xx[0:3] / 1000.0

        msg_pos = "直线终点\n" + \
                  "x1:" + str(xx[0]) + "\n" + "x2:" + str(xx[1]) + \
                  "\n" + "x3:" + str(xx[2]) + "\n" + "x4:" + str(xx[3]) + \
                  "\n" + "x5:" + str(xx[4]) + "\n" + "x6:" + str(xx[5]) + "\n"
        # 转化为m\s单位
        self.textEdit.setText(msg_pos)
        self.xx_e = np.copy(x_e)

    def plan(self):
        # 调用规划函数
        [xe, q] = gf.line_plan(self.qq_b, self.xx_b,
                               self.xx_e, self.T, self.t,
                               self.arm_flag)
        #更改为7维显示
        k1 = len(q[:, 0])
        qq = np.zeros([k1,7])
        qq[:, 0:self.n] = np.copy(q)

        # 调用绘图函数
        tt1 = np.linspace(0, self.T * (k1 - 1), k1)
        k2 = len(xe[:, 0])
        tt2 = np.linspace(0, self.T * (k2 - 1), k2)
        # 绘制关节角位置速度图
        self.plot_joint(tt1, qq, tt2, xe)
        # 将规划好的位置定义为全局变量
        self.qq_list = np.copy(qq)

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        # 正运动计算
        xx = gf.get_begin_point(qq, self.flag)

        #存储数据
        self.state_t = self.state_t + self.T
        self.state_qq_list.append(qq)
        self.state_t_list.append(self.state_t)
        self.state_xx_list.append(xx[0:3])
        # 仅记录1000个数据点
        del self.state_t_list[0]
        del self.state_qq_list[0]
        del self.state_xx_list[0]

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def probar_show(self):
        self.step_p = self.step_p + 1
        self.progressBar.setValue(self.step_p)
        if (self.step_p > 99):
            self.timer_p.stop()

    def off(self):
        self.flag = 0

    def realtime_plot(self):
        plot_t = np.array(self.state_t_list)
        plot_qq = np.array(self.state_qq_list)
        plot_xx = np.array(self.state_xx_list)
        self.plot_joint(plot_t, plot_qq, plot_t, plot_xx)

    def armc_or_ur5(self):
        # 运行该函数切换到ur5
        self.sub_path = "/robot1/joint_states"
        self.pub_path = "/robot1/ur5_position_controller/command"
        self.arm_flag = True
        self.n = 6

    def begin_function(self):
        # 运行标签启动
        self.flag = 1

        # 运行话题
        rospy.init_node('mainWindon_run!')
        rospy.Subscriber(self.sub_path, JointState, self.joint_callback)
        pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=10)

        # 求取数据长度
        kk = len(self.qq_list)

        #进度条显示时间间隔
        show_time = int(kk*self.T * 10)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg1 = "MainWindon run!;"
        self.textEdit.setText(msg1)
        t1.start()

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
            if (self.flag == 0):
                self.timer_plot.stop()
                break
            # 发送数据
            command_data = Float64MultiArray()
            if (k < kk):
                command_data.data = self.qq_list[k, 0:self.n]
            else:
                command_data.data = self.qq_list[-1, 0:self.n]
            pub.publish(command_data)
            if (k % 10 == 0):
                if (self.n == 7):
                    pub_msg = "armc" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                    "q1:" + str(
                        command_data.data[0]) + '\n' \
                                                "q2:" + str(command_data.data[1]) + '\n' \
                                                                                    "q3:" + str(
                        command_data.data[2]) + '\n' \
                                                "q4:" + str(command_data.data[3]) + '\n' \
                                                                                    "q5:" + str(
                        command_data.data[4]) + '\n' \
                                                "q6:" + str(command_data.data[5]) + '\n' \
                                                                                    "q7:" + str(
                        command_data.data[6]) + '\n'
                else:
                    pub_msg = "UR5" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                   "q1:" + str(
                        command_data.data[0]) + '\n' \
                                                "q2:" + str(command_data.data[1]) + '\n' \
                                                                                    "q3:" + str(
                        command_data.data[2]) + '\n' \
                                                "q4:" + str(command_data.data[3]) + '\n' \
                                                                                    "q5:" + str(
                        command_data.data[4]) + '\n' \
                                                "q6:" + str(command_data.data[5]) + '\n'
                self.textEdit.setText(pub_msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================示教窗口1================#
class TeachWindow1(QMainWindow, Ui_TeachingForm1):
    state_qq = np.zeros(7)
    state_f = np.zeros(6)
    state_t = 0

    def __init__(self, parent=None):
        super(TeachWindow1, self).__init__(parent)
        self.setupUi(self)
        self.initUI()

        self.t = 30
        self.T = 0.01

        self.sub_force_path = "/ft_sensor_topic"
        self.sub_pos_path = "/armc/joint_states"
        self.pub_path = "/armc/joint_positions_controller/command"
        self.n = 7  # 机械臂关节数

        #示教器标签
        self.flag = 1  # 开始或停止标签
        self.arm_flag = False
        self.run_flag = False
        self.teach_flag = False
        self.gazebo_flag = 0

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
        # 读取积分自适应阻抗参数MBKI
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_ready_teach.clicked.connect(self.ready_teaching)
        self.button_end_teach.clicked.connect(self.end_teaching)
        #self.button_teach_show.clicked.connect(self.show_msg)
        self.button_go_init.clicked.connect(self.go_init)
        self.button_go_home.clicked.connect(self.go_home)

        self.button_read_dir.clicked.connect(self.read_dir)
        self.button_teach_learn.clicked.connect(self.teaching_learn)

        self.button_plan_begin.clicked.connect(self.read_plan_begin)
        self.button_plan_end.clicked.connect(self.read_plan_end)

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
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)

    def go_init(self):
        self.init_flag = True
        self.imp_flag = False
        self.home_flag = False

        # 获得规划起点
        init_pos = np.array([0, 30, 0, 60, 0, 30, 0])
        qq_b = np.array(self.state_qq)
        # 调用规划函数
        [qq, qv, qa] = gf.q_joint_space_plan_time(qq_b, init_pos,
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

    def go_home(self):
        self.init_flag = False
        self.imp_flag = False
        self.home_flag = True

        # 获得规划起点
        qq_b = np.array(self.state_qq)
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

    # 计算初始位置
    def ready_teaching(self):
        #获取存储地址
        self.data_path = str(self.lineEdit_data_dir.text())
        # 建立示教器
        self.teaching = tl.teaching(self.data_path)
        msg = "成功创建示教器\n"
        self.textEdit_1.setText(msg)

        self.init_flag = False
        self.teach_flag = True
        self.home_flag = False

        msg = "已切换到示教模式！\n"
        self.textEdit.setText(msg)

    def end_teaching(self):
        self.teach_flag = False
        self.teaching.write_data()
        self.lineEdit_simple_data.setText(self.data_path)

    def read_dir(self):
        self.simple_data_path = str(self.lineEdit_simple_data.text())
        self.dmps_path = str(self.lineEdit_dmps_dir.text())
        self.rbf_path = str(self.lineEdit_rbf_dir.text())
        self.general_data_path = str(self.lineEdit_general_data.text())

        msg = "地址已经读取！"
        self.textEdit.setText(msg)

    def teaching_learn(self):
        #创建示教器
        teachingLearn = tl.TeachingLearn(self.n, self.dmps_path, self.rbf_path)

    def read_plan_begin(self):
        X = np.zeros(6)
        X[0] = self.lineEdit_x1.text()
        X[1] = self.lineEdit_x2.text()
        X[2] = self.lineEdit_x3.text()
        X[3] = self.lineEdit_x4.text()
        X[4] = self.lineEdit_x5.text()
        X[5] = self.lineEdit_x6.text()
        self.plan_end_pos = np.copy(X)
        msg = "规划初始点位置已经读取！"
        self.textEdit.setText(msg)

    def read_plan_end(self):
        X = np.zeros(6)
        X[0] = self.lineEdit_x1.text()
        X[1] = self.lineEdit_x2.text()
        X[2] = self.lineEdit_x3.text()
        X[3] = self.lineEdit_x4.text()
        X[4] = self.lineEdit_x5.text()
        X[5] = self.lineEdit_x6.text()
        self.plan_end_pos = np.copy(X)
        msg = "规划目标点位置已经读取！"
        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        # 存储数据
        self.state_qq = np.copy(qq)
        self.state_t = self.state_t + self.T

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
        self.state_f = np.copy(f)

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def stop(self):
        self.run_flag = False

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

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        # 创建阻抗自适应阻抗实例
        imp_arm1 = imp.IIMPController_iter()

        # 输入参数
        [DH0, q_max, q_min] = gf.get_robot_parameter(False)
        imp_arm1.get_robot_parameter(DH0, q_max, q_min, )
        imp_arm1.get_period(self.T)

        # 发送关节角度
        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                msg = "已停止运行！"
                self.textEdit.setText(msg)
                break

            if(self.teach_flag):
                # 实时调整阻抗参数
                imp_arm1.get_imp_parameter(self.M, self.B, self.K, self.I)
                # 读取期望位姿和关节角
                imp_arm1.get_expect_pos(self.xx_d)
                imp_arm1.get_current_joint(self.state_qq)
                # 读取当前关节角和力
                imp_arm1.get_expect_force(self.f_d)
                imp_arm1.get_current_force(self.state_f)
                # 计算修正关节角
                qr = imp_arm1.compute_imp_joint()
                # 发送数据
                command_data = Float64MultiArray()
                command_data.data = qr
                self.pub.publish(command_data)

                #加入示教数据
                self.teaching.get_joint_position(self.state_qq)
            # 显示数据
                if (k == 0):
                    msg = "阻抗开始！"
                    self.textEdit.setText(msg)
                else:
                    pass

            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================运行规划好的数据窗口================#
class RunDataWindow(QMainWindow, Ui_RunForm):
    # 建立全局变量,属性模块可以直接读取
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0
    T = 0.01
    flag = 1  # 开始或停止标签
    arm_flag = False
    gazebo_flag = 0
    sub_force_path = "/robot3/ft_sensor_topic"
    sub_pos_path = "/robot3/joint_states"
    pub_path = "/robot3/armc_position_controller/command"
    n = 7  # 机械臂关节数

    def __init__(self, parent=None):
        super(RunDataWindow, self).__init__(parent)
        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # ======================菜单栏功能模块=======================#
        # 创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        MainMenu = menubar.addMenu("Main")

        # 文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        MainMenu.addAction(openMain)

        # 文件菜单中打开文件操作
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
        self.button_end.clicked.connect(self.off)
        self.checkBox_UR5.stateChanged.connect(self.armc_or_ur5)
        self.button_get_dir.clicked.connect(self.read_path)

    #打开文件的地址和内容
    def fileOpen(self):
        #打开文件操作
        path = '/home/d/catkin_ws/src/robot_python/data'
        fname = QFileDialog.getOpenFileName(self, 'Open file', path)

        if fname[0]:
            f = open(fname[0], 'r')
            self.data_path = fname[0]
            self.lineEdit_data_path.setText(self.data_path)

            with f:
                data = f.read()
                self.pubdata = data
                self.textEdit.setText(data)

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

        p2 = win2.addPlot(title="F")  # 添加第一个绘图窗口
        p2.setLabel('left', text='force:N', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1, p2

    # 绘画关节角和关节角速度曲线
    def plot_joint(self, t1, qq, t2, ff):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)
        # 绘制操作空间位置图
        self.p2.plot(t2, ff[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t2, ff[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t2, ff[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t2, ff[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t2, ff[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t2, ff[:, 5], pen='y', name='F6', clear=False)

    def read_path(self):
        #获取地址
        self.data_path = str(self.lineEdit_data_path.text())
        self.sub_pos_path = str(self.lineEdit_sub_pos.text())
        self.sub_force_path = str(self.lineEdit_sub_force.text())
        self.pub_path = str(self.lineEdit_pub_path.text())

        # 读取数据
        self.qq_list = gf.read_data(self.data_path)

        #显示获取的地址
        msg = "数据和话题地址\n" + \
            "数据地址data_path：" + "\n" + self.data_path + "\n" + \
            "订阅关节角话题地址sub_pos_path:" + "\n" + self.sub_pos_path + "\n" + \
            "订阅六维力话题地址sub_force_path:" + "\n" + self.sub_force_path
        self.textEdit.setText(msg)

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

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        # 数据预处理
        qq_list = np.array(self.state_qq_list[-4:-1])
        # qq_p = gf.joint_pos_Pretreatment(qq, qq_list , self.T)
        qq_p = qq
        # 存储数据
        self.state_t = self.state_t + self.T
        self.state_qq_list.append(qq_p)
        self.state_t_list.append(self.state_t)
        # 仅记录1000个数据点
        del self.state_t_list[0]
        del self.state_qq_list[0]

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def probar_show(self):
        self.step_p = self.step_p + 1
        self.progressBar.setValue(self.step_p)
        if (self.step_p > 99):
            self.timer_p.stop()

    def off(self):
        self.flag = 0

    def realtime_plot(self):
        plot_t = np.array(self.state_t_list)
        plot_qq = np.array(self.state_qq_list)
        plot_f = np.array(self.state_f_list)
        self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

    def armc_or_ur5(self):
        # 运行该函数切换到ur5
        self.sub_pos_path = "/robot2/joint_states"
        self.sub_force_path = "/robot2/ft_sensor_topic"
        self.pub_path = "/robot2/ur5_position_controller/command"
        self.arm_flag = True
        self.n = 6

    def begin_function(self):
        # 初始化状态变量
        for i in range(3):
            self.state_qq_list[-i - 1] = self.qq_list[0, :]
        # 运行标签启动
        self.flag = 1

        # 运行话题
        rospy.init_node('mainWindon_run')
        rospy.Subscriber(self.sub_pos_path, JointState, self.joint_callback)
        rospy.Subscriber(self.sub_force_path, WrenchStamped, self.force_callback)
        pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=10)

        # 求取数据长度
        kk = len(self.qq_list)

        # 进度条显示时间间隔
        show_time = int(kk * self.T * 10)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg1 = "MainWindon run!;"
        self.textEdit.setText(msg1)
        t1.start()

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
            if (self.flag == 0):
                self.timer_plot.stop()
                self.timer_p.stop()
                break
            # 发送数据
            command_data = Float64MultiArray()
            if (k < kk):
                command_data.data = self.qq_list[k, 0:self.n]
            else:
                command_data.data = self.qq_list[-1, 0:self.n]
            pub.publish(command_data)
            if (k % 10 == 0):
                if (self.n == 7):
                    pub_msg = "armc" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                    "q1:" + str(
                        command_data.data[0]) + '\n' \
                                                "q2:" + str(command_data.data[1]) + '\n' \
                                                                                    "q3:" + str(
                        command_data.data[2]) + '\n' \
                                                "q4:" + str(command_data.data[3]) + '\n' \
                                                                                    "q5:" + str(
                        command_data.data[4]) + '\n' \
                                                "q6:" + str(command_data.data[5]) + '\n' \
                                                                                    "q7:" + str(
                        command_data.data[6]) + '\n'
                else:
                    pub_msg = "UR5" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                   "q1:" + str(
                        command_data.data[0]) + '\n' \
                                                "q2:" + str(command_data.data[1]) + '\n' \
                                                                                    "q3:" + str(
                        command_data.data[2]) + '\n' \
                                                "q4:" + str(command_data.data[3]) + '\n' \
                                                                                    "q5:" + str(
                        command_data.data[4]) + '\n' \
                                                "q6:" + str(command_data.data[5]) + '\n'
                self.textEdit.setText(pub_msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================数据采集测试窗口1================#
class TestWindow1(QMainWindow, Ui_TestForm1):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0
    flag = 1  # 开始或停止标签
    arm_flag = False
    gazebo_flag = 0
    sub_force_path = "/ft_sensor_topic"
    sub_pos_path = "/armc/joint_states"
    pub_path = "/armc/joint_positions_controller/command"
    n = 7  # 机械臂关节数

    def __init__(self, parent=None):
        super(TestWindow1, self).__init__(parent)
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
        self.button_begin.clicked.connect(self.begin_function)
        self.button_end.clicked.connect(self.off)
        self.button_plan.clicked.connect(self.plan)
        self.checkBox_UR5.stateChanged.connect(self.armc_or_ur5)
        self.button_plan_begin.clicked.connect(self.read_begin_point)
        self.button_plan_end.clicked.connect(self.read_end_point)

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

        p2 = win2.addPlot(title="F")  # 添加第一个绘图窗口
        p2.setLabel('left', text='force:N', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1, p2

    # 绘画关节角和关节角速度曲线
    def plot_joint(self, t1, qq, t2, ff):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t1, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t1, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t1, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t1, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t1, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t1, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t1, qq[:, 6], pen='w', name='qq7', clear=False)
        # 绘制操作空间位置图
        self.p2.plot(t2, ff[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t2, ff[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t2, ff[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t2, ff[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t2, ff[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t2, ff[:, 5], pen='y', name='F6', clear=False)

    # 计算初始位置
    def read_begin_point(self):
        # 初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()

        # 规划时间和周期
        self.T = float(self.lineEdit_T.text())
        self.t = float(self.lineEdit_t.text())

        msg_joint = "初始关节角\n" + \
                    "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                    "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                    "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                    "\n" + "q7:" + str(qq[6]) + "\n"

        msg_time = "规划时间和周期\n" + \
                   "T:" + str(self.T) + "\n" + "t:" + str(self.t) + "\n"

        msg = msg_joint + msg_time
        self.textEdit.setText(msg)
        # 转化为输入单位，并保存到全局变量
        self.qq_b = np.copy(qq * np.pi / 180.0)

    # 读取所有初始参数
    def read_end_point(self):
        # 初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()

        msg_joint = "终点关节角\n" + \
                    "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                    "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                    "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                    "\n" + "q7:" + str(qq[6]) + "\n"

        self.textEdit.setText(msg_joint)
        # 转化为输入单位，并保存到全局变量
        self.qq_e = np.copy(qq * np.pi / 180.0)

    def plan(self):
        # 调用规划函数
        [qq, qv_, T] = gf.q_joint_space_plan(self.qq_b, self.qq_e)
        # 调用绘图函数
        k = len(qq)
        t = np.linspace(0, T * (k - 1), k)
        self.T = T
        # 绘制关节角位置速度图
        self.plot_joint(t, qq, t , np.zeros([k, 6]))
        # 将规划好的位置定义为全局变量
        self.qq_list = np.copy(qq)

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

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        #数据预处理
        qq_list = np.array(self.state_qq_list[-4:-1])
        qq_p = gf.joint_pos_Pretreatment(qq, qq_list , self.T)
        #qq_p = qq
        #存储数据
        self.state_t = self.state_t + self.T
        self.state_qq_list.append(qq_p)
        self.state_t_list.append(self.state_t)
        # 仅记录1000个数据点
        del self.state_t_list[0]
        del self.state_qq_list[0]

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def probar_show(self):
        self.step_p = self.step_p + 1
        self.progressBar.setValue(self.step_p)
        if (self.step_p > 99):
            self.timer_p.stop()

    def off(self):
        self.flag = 0

    def realtime_plot(self):
        plot_t = np.array(self.state_t_list)
        plot_qq = np.array(self.state_qq_list)
        plot_f = np.array(self.state_f_list)
        self.plot_joint(plot_t, plot_qq, plot_t, plot_f)

    # 支持armt、armc四种状态切换
    def real_and_arm(self):
        self.real_flag = self.checkBox_real.isChecked()
        self.armt_flag = self.checkBox_arm.isChecked()
        if (self.real_flag):
            if (self.armt_flag):
                self.sub_force_path = "/armt/ft_sensor_topic"
                self.sub_pos_path = "/armt/joint_states"
                self.pub_path = "/armt/joint_command"
            else:
                self.sub_force_path = "/armc/ft_sensor_topic"
                self.sub_pos_path = "/joint_states"
                self.pub_path = "/all_joints_position_group_controller/command"
        else:
            if (self.armt_flag):
                self.sub_force_path = "/robot1/ft_sensor_topic"
                self.sub_pos_path = "/robot1/joint_states"
                self.pub_path = "/robot1/armt_position_controller/command"
            else:
                self.sub_force_path = "/ft_sensor_topic"
                self.sub_pos_path = "/armc/joint_states"
                self.pub_path = "/armc/joint_positions_controller/command"

    def begin_function(self):
        # 运行标签启动
        self.flag = 1

        # 运行话题
        rospy.init_node('mainWindon_run')
        rospy.Subscriber(self.sub_pos_path, JointState, self.joint_callback)
        rospy.Subscriber(self.sub_force_path, WrenchStamped, self.force_callback)
        pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=10)

        # 求取数据长度
        kk = len(self.qq_list)

        #进度条显示时间间隔
        show_time = int(kk*self.T * 10)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg1 = "MainWindon run!;"
        self.textEdit.setText(msg1)
        t1.start()

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
            if (self.flag == 0):
                self.timer_plot.stop()
                self.timer_p.stop()
                break
            # 发送数据
            command_data = Float64MultiArray()
            if (k < kk):
                command_data.data = self.qq_list[k, 0:self.n]
            else:
                command_data.data = self.qq_list[-1, 0:self.n]
            pub.publish(command_data)
            if (k % 10 == 0):
                if (self.n == 7):
                    pub_msg = "armc" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                    "q1:" + str(
                        command_data.data[0]) + '\n' \
                                                "q2:" + str(command_data.data[1]) + '\n' \
                                                                                    "q3:" + str(
                        command_data.data[2]) + '\n' \
                                                "q4:" + str(command_data.data[3]) + '\n' \
                                                                                    "q5:" + str(
                        command_data.data[4]) + '\n' \
                                                "q6:" + str(command_data.data[5]) + '\n' \
                                                                                    "q7:" + str(
                        command_data.data[6]) + '\n'
                else:
                    pub_msg = "UR5" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                                                                                   "q1:" + str(
                        command_data.data[0]) + '\n' \
                                                "q2:" + str(command_data.data[1]) + '\n' \
                                                                                    "q3:" + str(
                        command_data.data[2]) + '\n' \
                                                "q4:" + str(command_data.data[3]) + '\n' \
                                                                                    "q5:" + str(
                        command_data.data[4]) + '\n' \
                                                "q6:" + str(command_data.data[5]) + '\n'
                self.textEdit.setText(pub_msg)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()

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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MainWindow()
    myWin.show()
    sys.exit(app.exec_())