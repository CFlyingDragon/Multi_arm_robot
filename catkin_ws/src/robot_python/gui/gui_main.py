#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于GUI开发界面函数
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2020.4.21
#系统函数
import sys
import numpy as np
import time

#pyqt5函数
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

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
from main_windon import Ui_MainWindow
from urs_form1 import Ui_UrsForm1
from urs_form2 import Ui_UrsForm2
from urs_form3 import Ui_UrsForm3
from armct_form1 import Ui_ArmctForm1
from impedance_form1 import Ui_ImpForm1
from impedance_form2 import Ui_ImpForm2
from circularPlan_form1 import Ui_CirForm1
from linePlan_form1 import Ui_LineForm1
from rundata_form import Ui_RunForm
from test_form1 import Ui_TestForm1
from teaching_form1 import Ui_TeachingForm1

#自定义文件
import gui_function as gf
from robot_python import ImpedanceControl as imp
from robot_python import TeachingLearning as tl
from robot_python import FileOpen as fo

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
        self.gazebo_flag = False
        self.sub_force_path = "/ft_sensor_topic"
        self.sub_pos_path = "/armc/joint_states"
        self.pub_path = "/armc/joint_positions_controller/command"
        self.n = 7  # 机械臂关节数

        self.setupUi(self)
        self.initUI()
    def initUI(self):
        # ======================菜单栏功能模块=======================#
        #创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        ursMenu = menubar.addMenu('&3URs')
        armctMenu = menubar.addMenu('&Armc_Armt')
        planMenu = menubar.addMenu('&Plan')
        impedanceMenu = menubar.addMenu('Impedance')
        teachMenu = menubar.addMenu('&Teaching')
        demoMenu = menubar.addMenu('&Demo')
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

        # -------------------armct菜单-------------------#
        openArmct1 = QAction(QIcon('exit.png'), 'Open Armc_Armt Form1', self)
        openArmct1.setShortcut('Ctrl+c')
        openArmct1.setStatusTip('Open Armc_Armt joint plan form')
        openArmct1.triggered.connect(self.gotoArmct1)
        armctMenu.addAction(openArmct1)

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

        #阻抗控制菜单栏:阻抗参数调试
        openImp1 = QAction(QIcon('exit.png'), 'Open impedance1', self)
        openImp1.setStatusTip('Open impedance form1')
        openImp1.triggered.connect(self.gotoImp1)
        impedanceMenu.addAction(openImp1)

        #阻抗控制菜单栏:多臂阻抗控制
        openImp2 = QAction(QIcon('exit.png'), 'Open impedance2', self)
        openImp2.setStatusTip('Open impedance form2')
        openImp2.triggered.connect(self.gotoImp2)
        impedanceMenu.addAction(openImp2)

        #示教菜单栏:
        openteach1 = QAction(QIcon('exit.png'), 'Open teaching1', self)
        openteach1.setStatusTip('Open teaching1 form1')
        openteach1.triggered.connect(self.gototeach1)
        teachMenu.addAction(openteach1)

        #案例菜单栏：运行给定数据
        openData = QAction(QIcon('exit.png'), 'Open run data', self)
        openData.setStatusTip('Open run data form')
        openData.triggered.connect(self.gotoData)
        demoMenu.addAction(openData)
        #测试菜单栏
        openTest1 = QAction(QIcon('exit.png'), 'Open test1', self)
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
            command_data = Float64MultiArray()
            if (k < kk):
                command_data.data = self.command_qq_list[k, 0:self.n]
            else:
                command_data.data = self.command_qq_list[-1, 0:self.n]
            self.pub.publish(command_data)
            if(k%10==0):
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
            [qq,qv,qa] = gf.q_joint_space_plan_time(qq_b1, self.qq_wish_pos[0, :])
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

# ================三个UR5协同控制================#
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

        if(self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq1, qv, qa] = gf.q_joint_space_plan_time(qq_b1, self.robot1_command_qq[0, :],
                                                      self.T, self.t)
            # 调用绘图函数
            k1 = len(qq1[:, 0])
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            # 绘制关节角位置图
            self.plot_joint(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_init = np.copy(qq1)

        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2, qv, qa] = gf.q_joint_space_plan_time(qq_b2, self.robot2_command_qq[0, :],
                                                      self.T, self.t)
            # 调用绘图函数
            k2 = len(qq2[:, 0])
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
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

        if (self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b1 = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq1, qv, qa] = gf.q_joint_space_plan_time(qq_b1, qq_home, self.T, self.t)
            # 调用绘图函数
            k1 = len(qq1[:, 0])
            t1 = np.linspace(0, self.T * (k1 - 1), k1)
            # 绘制关节角位置图
            self.plot_joint(t1, qq1)
            # 将规划好的位置定义为全局变量
            self.command_qq1_home = np.copy(qq1)

        if (self.robot2_flag or self.all_flag):
            # 获得规划起点
            qq_b2 = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq2, qv, qa] = gf.q_joint_space_plan_time(qq_b2, qq_home, self.T, self.t)
            # 调用绘图函数
            k2 = len(qq2[:, 0])
            t2 = np.linspace(0, self.T * (k2 - 1), k2)
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
            qq = np.array([-90, -60, -120, 60, 90, 0])
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

# ================三个UR5协同控制================#
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

        self.robot1_2_flag = False
        self.robot2_2_flag = False

        self.sub_force1_path = "/robot1/ft_sensor_topic"
        self.sub_pos1_path = "/robot1/joint_states"
        self.pub1_path = "/robot1/armt_position_controller/command"
        self.sub_force2_path = "/robot2/ft_sensor_topic"
        self.sub_pos2_path = "/robot2/joint_states"
        self.pub2_path = "/robot2/armc_position_controller/command"

        self.n = 7  # 机械臂关节数
        self.qq_wish_pos = np.zeros([2, 7])

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
        self.p1.plot(t1, qq[:, 6], pen='k', name='qq7', clear=False)

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
        self.all_flag = self.radioButton_all.isChecked()

        self.robot1_2_flag = self.radioButton_robot1_2.isChecked()
        self.robot2_2_flag = self.radioButton_robot2_2.isChecked()

        self.real_flag = self.radioButton_real.isChecked()
        self.gazebo_flag = self.radioButton_gazebo.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

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

        msg = ""
        if (self.robot1_flag or self.all_flag):
            msg_pos1 = "robot1规划目标点:\n" + \
                       "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                       "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                       "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                       "\n" + "q7:" + str(qq[6]) + "\n"
            self.qq_wish_pos[0, :] = np.copy(qq * np.pi / 180.0)
            msg = msg + msg_pos1

        if (self.robot2_flag or self.all_flag):
            msg_pos = "robot2规划目标点:\n" + \
                      "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                      "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                      "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                      "\n" + "q7:" + str(qq[6]) + "\n"
            self.qq_wish_pos[1, :] = np.copy(qq * np.pi / 180.0)
            msg = msg + msg_pos

            msg = msg + msg_pos

        if (not (self.robot1_flag or self.robot2_flag or self.all_flag)):
            msg = "没有选中机械臂！！！"

        self.textEdit.setText(msg)

    # 给定初始位置
    def init(self):
        qq = np.array([0, -30, 0, 90, 0, 30, 0])
        self.lineEdit_q1.setText(str(qq[0]))
        self.lineEdit_q2.setText(str(qq[1]))
        self.lineEdit_q3.setText(str(qq[2]))
        self.lineEdit_q4.setText(str(qq[3]))
        self.lineEdit_q5.setText(str(qq[4]))
        self.lineEdit_q6.setText(str(qq[5]))
        self.lineEdit_q7.setText(str(qq[6]))

    # 给定家点位置
    def home(self):
        qq = np.array([0, 0, 0, 0, 0, 0, 0])
        self.lineEdit_q1.setText(str(qq[0]))
        self.lineEdit_q2.setText(str(qq[1]))
        self.lineEdit_q3.setText(str(qq[2]))
        self.lineEdit_q4.setText(str(qq[3]))
        self.lineEdit_q5.setText(str(qq[4]))
        self.lineEdit_q6.setText(str(qq[5]))
        self.lineEdit_q7.setText(str(qq[6]))

    # 规划函数
    def plan(self):
        msg = ""
        if (self.robot1_flag or self.all_flag):
            # 获得规划起点
            qq_b = np.array(self.state_qq_list1[-1])
            # 调用规划函数
            [qq, qv, qa] = gf.q_joint_space_plan_time(qq_b, self.qq_wish_pos[0, :])
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
            # 获得规划起点
            qq_b = np.array(self.state_qq_list2[-1])
            # 调用规划函数
            [qq, qv, qa] = gf.q_joint_space_plan_time(qq_b, self.qq_wish_pos[1, :])
            # 调用绘图函数
            k = len(qq[:, 0])
            t = np.linspace(0, self.T * (k - 1), k)
            # 绘制关节角位置速度图
            self.plot_joint(t, qq, t, np.zeros([k, 6]))
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

    def run_topic(self):
        # 读取时间
        self.T = float(self.lineEdit_T.text())
        self.t = float(self.lineEdit_t.text())

        msg_time = "获取时间：\n" + "周期T：" + str(self.T) + \
                   "\n规划时长：" + str(self.t) + "\n"

        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos1_path, JointState, self.joint_callback1)
        rospy.Subscriber(self.sub_force1_path, JointState, self.force_callback1)
        self.pub1 = rospy.Publisher(self.pub1_path, Float64MultiArray, queue_size=100)

        rospy.Subscriber(self.sub_pos2_path, JointState, self.joint_callback2)
        rospy.Subscriber(self.sub_force2_path, JointState, self.force_callback2)
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

# ================阻抗控制窗口1================#
class ImpWindow1(QMainWindow, Ui_ImpForm1):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0

    def __init__(self, parent=None):
        super(ImpWindow1, self).__init__(parent)
        self.T = 0.01

        self.run_flag = True  # 开始或停止标签
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
        self.button_stop.clicked.connect(self.stop)
        self.button_init.clicked.connect(self.calculation_init_point)
        self.checkBox_real.stateChanged.connect(self.real_arm)
        self.button_read.clicked.connect(self.read_paramter)
        self.button_receive.clicked.connect(self.run_topic)

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

    def real_arm(self):
        self.real_flag = True

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
        xx_b = gf.get_begin_point(qq, False)

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

        #读取期望末端位置
        xx = np.zeros(6)
        xx[0] = self.lineEdit_x1.text()
        xx[1] = self.lineEdit_x2.text()
        xx[2] = self.lineEdit_x3.text()
        xx[3] = self.lineEdit_x4.text()
        xx[4] = self.lineEdit_x5.text()
        xx[5] = self.lineEdit_x6.text()
        #改变为输入单位
        x_d = np.copy(xx)
        x_d[0:3] = xx[0:3] / 1000.0

        #读取期望末端力,工具坐标系中描述
        f = np.zeros(6)
        f[0] = self.lineEdit_f1.text()
        f[1] = self.lineEdit_f2.text()
        f[2] = self.lineEdit_f3.text()
        f[3] = self.lineEdit_f4.text()
        f[4] = self.lineEdit_f5.text()
        f[5] = self.lineEdit_f6.text()

        #读取阻抗参数
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
        self.qq_b = np.copy(qq * np.pi / 180.0)
        self.xx_d = np.copy(x_d)
        self.f_d = np.copy(f)
        self.M = np.copy(M)
        self.B = np.copy(B)
        self.K = np.copy(K)
        self.I = np.copy(I)

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

    def begin_function(self):
        # 运行标签启动
        self.run_flag = True

        # 创建阻抗自适应阻抗实例
        imp_arm1 = imp.IIMPController_iter()

        #输入参数
        [DH0, q_max, q_min] = gf.get_robot_parameter(False)
        imp_arm1.get_robot_parameter(DH0, q_max, q_min,)
        imp_arm1.get_period(self.T)

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
        k = 0
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                # self.timer_plot.stop()
                # self.timer_p.stop()
                break
            t1 = time.time()
            #实时调整阻抗参数
            imp_arm1.get_imp_parameter(self.M, self.B, self.K, self.I)
            #读取期望位姿和关节角
            imp_arm1.get_expect_pos(self.xx_d)
            imp_arm1.get_current_joint(self.qq_state)
            #读取当前关节角和力
            imp_arm1.get_expect_force(self.f_d)
            imp_arm1.get_current_force(self.f_state)
            #计算修正关节角
            qr = imp_arm1.compute_imp_joint()
            # 发送数据
            command_data = Float64MultiArray()
            command_data.data = qr
            self.pub.publish(command_data)
            #显示数据

            if (k % 10 == 0):
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
            t2 = time.time()
            print "阻抗时间：", (t2 - t1)
            rate.sleep()

    def gotoMain(self):
        self.hide()
        self.main_windon = MainWindow()
        self.main_windon.show()

# ================阻抗控制窗口2================#
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
        mainMenu = menubar.addMenu('&Main')

        #文件菜单:返回主窗口
        openMain = QAction(QIcon('exit.png'), 'main window ', self)
        openMain.setShortcut('Ctrl+z')
        openMain.setStatusTip('Return main window')
        openMain.triggered.connect(self.gotoMain)
        mainMenu.addAction(openMain)

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        # 读取积分自适应阻抗参数MBKI
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_init.clicked.connect(self.go_init)
        self.button_imp.clicked.connect(self.go_imp)
        self.button_end.clicked.connect(self.go_home)
        self.checkBox_gazebo.stateChanged.connect(self.real_arm)
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

    def real_arm(self):
        self.real_flag = True

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
        self.plot_joint(t, np.zeros([k, 7]), t, self.command_force)

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
        self.plot_joint(t, self.command_qq_imp, t, np.zeros([k, 6]))

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

        #提示标语
        msg = "开始下发命令！\n"
        self.textEdit.setText(msg)
        # 创建阻抗自适应阻抗实例
        imp_arm1 = imp.IIMPController_iter()

        #获取阻抗参数
        self.M = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.B = np.array([0.0, 0.0, 200.0, 0.0, 0.0, 0.0])
        self.K = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.I = np.array([0.0, 0.0, 5, 0.0, 0.0, 0.0])

        # 输入参数
        [DH0, q_max, q_min] = gf.get_robot_parameter(False)
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
        kk1 = 0
        k1 = 0
        kk2 = 0
        k2 =0
        kk3 = 0
        k3 = 0
        if(self.init_flag):
            kk1 = len(self.command_qq_init)
            k1 = 0
        if(self.imp_flag):
            kk2 = len(self.command_qq_imp)
            k2 = 0
        if(self.home_flag):
            kk3 = len(self.command_qq_home)
            k3 = 0

        while not rospy.is_shutdown():
            # 检测是否启动急停
            if (not self.run_flag):
                # self.timer_plot.stop()
                # self.timer_p.stop()
                break

            if(self.init_flag):
                command_data = Float64MultiArray()
                if (k1 < kk1):
                    command_data.data = self.command_qq_init[k1, 0:self.n]
                else:
                    command_data.data = self.command_qq_init[-1, 0:self.n]
                self.pub.publish(command_data)
                if (k1==0):
                    msg = "运动到初始位置已经开始！"
                    self.textEdit.setText(msg)
                k1 = k1 + 1

            if(self.imp_flag):
                qq = np.zeros(7)
                fd = np.zeros(6)
                if (k2 < kk2):
                    qq = self.command_qq_imp[k2, 0:self.n]
                    fd = self.command_force[k2, :]
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
                command_data = Float64MultiArray()
                command_data.data = qr
                self.pub.publish(command_data)

                if (k2==0):
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
    sub_force_path = "/robot3/ft_sensor_topic"
    sub_pos_path = "/robot3/joint_states"
    pub_path = "/robot3/armc_position_controller/command"
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

    def armc_or_ur5(self):
        # 运行该函数切换到ur5
        self.sub_pos_path = "/robot2/joint_states"
        self.sub_force_path = "/robot2/ft_sensor_topic"
        self.pub_path = "/robot2/ur5_position_controller/command"
        self.arm_flag = True
        self.n = 6

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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MainWindow()
    myWin.show()
    sys.exit(app.exec_())

