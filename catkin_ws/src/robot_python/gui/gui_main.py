#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于GUI开发界面函数：调用其他子函数
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020.4.21

#文件分开存储
from gui_main_sub1 import *
from gui_main_sub2 import *
from gui_main_sub3 import *

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
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from robot_msg.srv import *

#绘图函数
import pyqtgraph as pg

#线程函数
import threading

#界面函数
from main_windon import Ui_MainWindow
from armc_form1 import Ui_ArmcForm1
from armc_form2 import Ui_ArmcForm2
from armc_form3 import Ui_ArmcForm3
from armc_form4 import Ui_ArmcForm4
from circularPlan_form1 import Ui_CirForm1
from linePlan_form1 import Ui_LineForm1
from rundata_form import Ui_RunForm
from test_form1 import Ui_TestForm1
from teaching_form1 import Ui_TeachingForm1

#自定义文件
import gui_function as gf
from robot_python import ImpedanceControl as imp
from robot_python import TeachingLearning as tl
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
        fileMenu = menubar.addMenu('&文件')
        plotMenu = menubar.addMenu('&绘图')
        armcMenu = menubar.addMenu('&自制Armc')
        ursMenu = menubar.addMenu('&UR5三臂')
        handMenu = menubar.addMenu('&手抓')
        armctMenu = menubar.addMenu('&双臂Armct')
        armctrMenu = menubar.addMenu('&三臂Armrtc')
        planMenu = menubar.addMenu('&规划')
        impedanceMenu = menubar.addMenu('阻抗控制')
        teachMenu = menubar.addMenu('&示教')
        demoMenu = menubar.addMenu('&工艺')
        testMenu = menubar.addMenu('&测试')
        helpMenu = menubar.addMenu('&帮助')

        #-------------------文件菜单-------------------#
        # 中打开文件操作
        openFile = QAction(QIcon('exit.png'), '打开', self)
        openFile.setShortcut('Ctrl+o')
        openFile.setStatusTip('打开文件')
        openFile.triggered.connect(self.fileOpen)
        fileMenu.addAction(openFile)

        #文件菜单中关闭操作
        exitAction = QAction(QIcon('exit.png'), '&退出', self)
        exitAction.setShortcut('Ctrl+q')
        exitAction.setStatusTip('退出软件')
        exitAction.triggered.connect(qApp.quit)
        fileMenu.addAction(exitAction)

        # -------------------绘图菜单-------------------#
        # armctr绘图
        openPlotArmctr = QAction(QIcon('exit.png'), 'Armctr绘图', self)
        openPlotArmctr.setShortcut('Ctrl+c')
        openPlotArmctr.setStatusTip('三臂场景绘图')
        openPlotArmctr.triggered.connect(self.gotoPlotArmctr)
        plotMenu.addAction(openPlotArmctr)

        # -------------------Armc菜单-------------------#
        #armc机械臂关节空间规划
        openArmc1 = QAction(QIcon('exit.png'), '关节控制', self)
        openArmc1.setShortcut('Ctrl+c')
        openArmc1.setStatusTip('自制机械臂关节空间规划和控制')
        openArmc1.triggered.connect(self.gotoArmc1)
        armcMenu.addAction(openArmc1)

        # armc机械臂增量规划
        openArmc2 = QAction(QIcon('exit.png'), '笛卡尔增量控制', self)
        openArmc2.setShortcut('Ctrl+c')
        openArmc2.setStatusTip('常规求逆控制')
        openArmc2.triggered.connect(self.gotoArmc2)
        armcMenu.addAction(openArmc2)

        # armc机械臂增量规划
        openArmc3 = QAction(QIcon('exit.png'), '笛卡尔零控制', self)
        openArmc3.setShortcut('Ctrl+c')
        openArmc3.setStatusTip('臂型角求逆控制')
        openArmc3.triggered.connect(self.gotoArmc3)
        armcMenu.addAction(openArmc3)

        # armc机械臂最终规划
        openArmc4 = QAction(QIcon('exit.png'), '推荐笛卡尔控制', self)
        openArmc4.setShortcut('Ctrl+c')
        openArmc4.setStatusTip('常规求逆零空间控制')
        openArmc4.triggered.connect(self.gotoArmc4)
        armcMenu.addAction(openArmc4)

        # -------------------URs菜单-------------------#
        #3个UR机械臂关节空间规划
        openUrs1 = QAction(QIcon('exit.png'), '关节空间控制', self)
        openUrs1.setShortcut('Ctrl+c')
        openUrs1.setStatusTip('三个UR机械臂关节空间规划')
        openUrs1.triggered.connect(self.gotoUrs1)
        ursMenu.addAction(openUrs1)

        #2个UR机械臂搬运物体
        openUrs2 = QAction(QIcon('exit.png'), '搬运控制', self)
        openUrs2.setShortcut('Ctrl+c')
        openUrs2.setStatusTip('三个UR机械臂搬运控制')
        openUrs2.triggered.connect(self.gotoUrs2)
        ursMenu.addAction(openUrs2)

        # 3个UR机械臂初始位置调整
        openUrs3 = QAction(QIcon('exit.png'), '初始位置调整', self)
        openUrs3.setShortcut('Ctrl+c')
        openUrs3.setStatusTip('调整UR机械臂初始位置')
        openUrs3.triggered.connect(self.gotoUrs3)
        ursMenu.addAction(openUrs3)

        # -------------------hand菜单-------------------#
        #URs加手抓
        openHand1 = QAction(QIcon('exit.png'), 'UR+手抓', self)
        openHand1.setShortcut('Ctrl+c')
        openHand1.setStatusTip('UR加手抓控制')
        openHand1.triggered.connect(self.gotoHand1)
        handMenu.addAction(openHand1)

        # -------------------armct菜单-------------------#
        openArmct1 = QAction(QIcon('exit.png'), '关节空间控制', self)
        openArmct1.setShortcut('Ctrl+c')
        openArmct1.setStatusTip('关节空间规划')
        openArmct1.triggered.connect(self.gotoArmct1)
        armctMenu.addAction(openArmct1)

        openArmct2 = QAction(QIcon('exit.png'), '笛卡尔空间控制', self)
        openArmct2.setShortcut('Ctrl+c')
        openArmct2.setStatusTip('笛卡尔空间规划')
        openArmct2.triggered.connect(self.gotoArmct2)
        armctMenu.addAction(openArmct2)

        openArmct3 = QAction(QIcon('exit.png'), '阻抗控制', self)
        openArmct3.setShortcut('Ctrl+c')
        openArmct3.setStatusTip('笛卡尔空间阻抗控制')
        openArmct3.triggered.connect(self.gotoArmct3)
        armctMenu.addAction(openArmct3)

        # -------------------armctr菜单-------------------#
        openArmctr1 = QAction(QIcon('exit.png'), '三臂阻抗控制', self)
        openArmctr1.setShortcut('Ctrl+c')
        openArmctr1.setStatusTip('阻抗控制')
        openArmctr1.triggered.connect(self.gotoArmctr1)
        armctrMenu.addAction(openArmctr1)

        openArmctr2 = QAction(QIcon('exit.png'), '三臂关节控制', self)
        openArmctr2.setShortcut('Ctrl+c')
        openArmctr2.setStatusTip('关节空间控制')
        openArmctr2.triggered.connect(self.gotoArmctr2)
        armctrMenu.addAction(openArmctr2)

        # openArmct3 = QAction(QIcon('exit.png'), 'imp plan form', self)
        # openArmct3.setShortcut('Ctrl+c')
        # openArmct3.setStatusTip('笛卡尔空间规划')
        # openArmct3.triggered.connect(self.gotoArmct3)
        # armctMenu.addAction(openArmct3)

        #规划菜单栏:打开圆规划
        openCirPlan = QAction(QIcon('exit.png'), '圆规划', self)
        openCirPlan.setShortcut('Ctrl+c')
        openCirPlan.setStatusTip('圆规划')
        openCirPlan.triggered.connect(self.gotoCircular1)
        planMenu.addAction(openCirPlan)

        #规划菜单栏:打直线规划规划
        openLinePlan = QAction(QIcon('exit.png'), '直线规划', self)
        openLinePlan.setShortcut('Ctrl+l')
        openLinePlan.setStatusTip('直线规划')
        openLinePlan.triggered.connect(self.gotoLine1)
        planMenu.addAction(openLinePlan)

        # -------------------积分自适应导纳菜单-------------------#
        #阻抗控制菜单栏:阻抗参数调试
        openImp1 = QAction(QIcon('exit.png'), '阻抗参数调试 ', self)
        openImp1.setStatusTip('阻抗参数调试')
        openImp1.triggered.connect(self.gotoImp1)
        impedanceMenu.addAction(openImp1)

        #阻抗控制菜单栏:阻抗轨迹控制
        openImp2 = QAction(QIcon('exit.png'), '单臂阻抗控制', self)
        openImp2.setStatusTip('单臂阻抗控制')
        openImp2.triggered.connect(self.gotoImp2)
        impedanceMenu.addAction(openImp2)

        #阻抗控制菜单栏:等效刚度评估
        openImp3 = QAction(QIcon('exit.png'), '刚度评估控制', self)
        openImp3.setStatusTip('刚度评估控制')
        openImp3.triggered.connect(self.gotoImp3)
        impedanceMenu.addAction(openImp3)

        # 阻抗控制菜单栏:笛卡尔空间阻抗控制
        openImp4 = QAction(QIcon('exit.png'), '笛卡尔控制阻抗控制', self)
        openImp4.setStatusTip('笛卡尔空间阻抗控制')
        openImp4.triggered.connect(self.gotoImp4)
        impedanceMenu.addAction(openImp4)

        # ----------------------示教菜单栏------------------------#
        #示教菜单栏:
        openteach1 = QAction(QIcon('exit.png'), 'Open openTeachn1', self)
        openteach1.setStatusTip('Open teach form1')
        openteach1.triggered.connect(self.gototeach1)
        teachMenu.addAction(openteach1)

        #----------------------工艺菜单栏------------------------#
        # 运行给定数据
        openTechn1 = QAction(QIcon('exit.png'), '打磨工艺', self)
        openTechn1.setStatusTip('打磨工艺')
        openTechn1.triggered.connect(self.gotoTechnology)
        demoMenu.addAction(openTechn1)

        #运行给定数据
        openData = QAction(QIcon('exit.png'), '数据控制', self)
        openData.setStatusTip('数据控制')
        openData.triggered.connect(self.gotoData)
        demoMenu.addAction(openData)

        # ----------------------测试菜单栏------------------------#
        #测试菜单栏
        openTest1 = QAction(QIcon('exit.png'), '滤波测试', self)
        openTest1.setStatusTip('运行滤波函数')
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

    def gotoImp4(self):
        self.hide()
        self.sub_imp4 = ImpWindow4()
        self.sub_imp4.show()

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

    def gotoArmctr1(self):
        self.hide()
        self.armctr1 = ArmctrWindow1()
        self.armctr1.show()

    def gotoArmctr2(self):
        self.hide()
        self.armctr2 = ArmctrWindow2()
        self.armctr2.show()

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

    def gotoArmc3(self):
        self.hide()
        self.armc3 = ArmcWindow3()
        self.armc3.show()

    def gotoArmc4(self):
        self.hide()
        self.armc4 = ArmcWindow4()
        self.armc4.show()

    def gotoPlotArmctr(self):
        self.hide()
        self.plotArmctr = PlotArmctrWindow()
        self.plotArmctr.show()

#***************************armc窗口1：关节空间规划********************************#
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

# ================armc窗口2:笛卡尔空间规划================#
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

# ================armc窗口3:零空间规划================#
class ArmcWindow3(QMainWindow, Ui_ArmcForm3):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0

    def __init__(self, parent=None):
        super(ArmcWindow3, self).__init__(parent)
        self.T = 0.01
        self.t = 10

        self.run_flag = False  # 开始或停止标签
        self.run_finish = True
        self.real_flag = False
        self.joint_flag = False

        self.pos1 = False
        self.pos2 = False

        self.read_flag = False
        self.visual_flag = False

        [DH0, q_max, q_min] = gf.get_robot_parameter("armc")
        self.DH0 = DH0
        self.q_max = q_max
        self.q_min = q_min
        self.robot = "armc"
        # 创建运动学求解器
        self.my_kin = kin.GeneralKinematic(self.DH0, self.q_min, self.q_max)

        # 建立规划类
        self.line_plan = pap.ArmcLinePlanZeros()
        self.line_plan.get_period(self.T)
        self.line_plan.get_robot_parameter(DH0, q_min, q_max)

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
        self.button_read.clicked.connect(self.read_qq)
        self.checkBox_real.stateChanged.connect(self.real_and_joint)
        self.button_get_init.clicked.connect(self.calculation_init_point_xx)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_init.clicked.connect(self.go_init)
        self.button_inc.clicked.connect(self.go_inc)
        self.button_home.clicked.connect(self.go_home)

        # -------增量控制按钮栏--------#
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
        self.button_xx7_N.clicked.connect(self.fun_xx7_n)
        self.button_xx1_P.clicked.connect(self.fun_xx1_p)
        self.button_xx2_P.clicked.connect(self.fun_xx2_p)
        self.button_xx3_P.clicked.connect(self.fun_xx3_p)
        self.button_xx4_P.clicked.connect(self.fun_xx4_p)
        self.button_xx5_P.clicked.connect(self.fun_xx5_p)
        self.button_xx6_P.clicked.connect(self.fun_xx6_p)
        self.button_xx7_P.clicked.connect(self.fun_xx7_p)

        self.button_check.clicked.connect(self.visual_check1)
        self.button_h_go.clicked.connect(self.handle_go_init)
        self.button_h_run.clicked.connect(self.handle_run)
        self.button_h_back.clicked.connect(self.handle_back_init)
        self.button_l_go.clicked.connect(self.lock_go_init)
        self.button_l_run.clicked.connect(self.lock_run)
        self.button_l_back.clicked.connect(self.lock_back_init)

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

    #调用视觉检测
    def visual_check(self):
        #读取检测目标种类
        flag = self.radioButton.isChecked()
        if(flag):
            a = 1
        else:
            a = 0
        rospy.wait_for_service('visual_inspection')
        try:
            visual = rospy.ServiceProxy('visual_inspection', VisualVar)
            resp1 = visual(a)
            print "resp1:", resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        #位置是否检测成功
        self.visual_flag = resp1.flag
        if(self.visual_flag):
            if(a==1):
                handle_X = np.array(resp1.T[0:6])
                R = self.my_kin.euler_zyx2rot(handle_X[3:6])
                Tc_o = np.eye(4)
                Tc_o[0:3, 0:3] = R.T
                Tc_o[0:3, 3] = np.dot(-R.T, handle_X[0:3])
                Te_c = gf.camera_parameters()
                T0_e = self.my_kin.fkine(self.qq_state)
                T0_o = np.dot(np.dot(T0_e, Te_c), Tc_o)

                handle_X[0:3] = T0_o[0:3, 3]
                handle_X[3:6] = self.my_kin.rot2euler_zyx(T0_o[0:3, 0:3])
                self.handle_X = handle_X

                msg = "门把手检测成功！"
                msg = msg + "\nx:" + str(handle_X[0]) + "\ny:" + str(handle_X[1]) \
                      + "\nz:" + str(handle_X[2]) + "\nR:" + str(handle_X[3]) \
                      + "\nP:" + str(handle_X[4]) + "\nY:" + str(handle_X[5])
            else:
                To_c = np.eye(4)
                for i in range(12):
                    To_c[i/4, 1%4] = resp1.T[0:6]
                Tc_o = np.inv(To_c)
                Te_c = gf.camera_parameters()
                T0_e = self.my_kin.fkine(self.qq_state)
                T0_o = np.dot(np.dot(T0_e, Te_c), Tc_o)
                lock_X = np.zeros(6)
                lock_X[0:3] = T0_o[0:3, 3]
                lock_X[3:6] = self.my_kin.rot2euler_zyx(T0_o[0:3, 0:3])
                self.lock_X = lock_X
                msg = "密码锁检测成功！"
                msg = msg + "\nx:" + str(lock_X[0]) + "\ny:" + str(lock_X[1]) \
                      + "\nz:" + str(lock_X[2]) + "\nR:" + str(lock_X[3]) \
                      + "\nP:" + str(lock_X[4]) + "\nY:" + str(lock_X[5])
        else:
            msg = "检测失败！！！"
        self.textEdit.setText(msg)

    def visual_check1(self):
        # 读取检测目标种类
        flag = self.radioButton.isChecked()
        self.visual_flag = True
        self.handle_X = np.array([0.800, -0.26, 0.220, 0 , 0, 0])
        self.lock_X = np.array([0.800, -0.26, 0.220, 0 , 0, 0])
        msg = "检测成功！"
        self.textEdit.setText(msg)

    # 计算初始位置
    def calculation_init_point_xx(self):
        # 初始关节角
        qq = np.array([-60.0, 30, 50.0, 40, -20, -20.0, 0])
        self.lineEdit_qq1.setText(str(qq[0]))
        self.lineEdit_qq2.setText(str(qq[1]))
        self.lineEdit_qq3.setText(str(qq[2]))
        self.lineEdit_qq4.setText(str(qq[3]))
        self.lineEdit_qq5.setText(str(qq[4]))
        self.lineEdit_qq6.setText(str(qq[5]))
        self.lineEdit_qq7.setText(str(qq[6]))

        qq[0] = self.lineEdit_qq1.text()
        qq[1] = self.lineEdit_qq2.text()
        qq[2] = self.lineEdit_qq3.text()
        qq[3] = self.lineEdit_qq4.text()
        qq[4] = self.lineEdit_qq5.text()
        qq[5] = self.lineEdit_qq6.text()
        qq[6] = self.lineEdit_qq7.text()

        # 转换单位
        qq = qq * np.pi / 180.0

        # 计算初始位置
        xx_b = self.my_kin.fkine_zeros(qq)

        # 转换到显示单位
        xx = np.copy(xx_b)  # 转化为mm显示
        xx[0:3] = xx_b[0:3] * 1000  # 转换为mm显示
        xx[3:] = xx_b[3:] * 180.0 / np.pi  # 转换为mm显示

        # 显示到界面
        self.lineEdit_xx1.setText(str(round(xx[0], 4)))
        self.lineEdit_xx2.setText(str(round(xx[1], 4)))
        self.lineEdit_xx3.setText(str(round(xx[2], 4)))
        self.lineEdit_xx4.setText(str(round(xx[3], 4)))
        self.lineEdit_xx5.setText(str(round(xx[4], 4)))
        self.lineEdit_xx6.setText(str(round(xx[5], 4)))
        self.lineEdit_xx7.setText(str(round(xx[6], 4)))
        msg = "初始末端位置\n" + \
              "x1:" + str(xx[0]) + "\n" + "x2:" + str(xx[1]) + \
              "\n" + "x3:" + str(xx[2]) + "\n" + "x4:" + str(xx[3]) + \
              "\n" + "x5:" + str(xx[4]) + "\n" + "x6:" + str(xx[5]) + "\n"\
                + "psi:" + str(xx[6]) + "\n"
        self.textEdit.setText(msg)

    def read_qq(self):
        # 初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_qq1.text()
        qq[1] = self.lineEdit_qq2.text()
        qq[2] = self.lineEdit_qq3.text()
        qq[3] = self.lineEdit_qq4.text()
        qq[4] = self.lineEdit_qq5.text()
        qq[5] = self.lineEdit_qq6.text()
        qq[6] = self.lineEdit_qq7.text()

        # 转换单位
        qq1 = qq * np.pi / 180.0
        self.qq_init = np.copy(qq1)

        msg = "读取目标关节角:\n" + \
              "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + "\n" + \
              "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + "\n" + \
              "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + "\n" + \
              "q7:" + str(qq[6]) + "\n"
        self.textEdit.setText(msg)

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
        self.command_qq = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    def go_inc(self):
        xx = self.my_kin.fkine_zeros(self.qq_state)
        self.XX = np.copy(xx)
        self.qq = np.copy(self.qq_state)
        qq = self.qq_state * 180 / np.pi
        qq = np.around(qq, 2)
        xx[:3] = xx[:3] * 1000
        xx[3:] = xx[3:] * 180 / np.pi
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
        self.lineEdit_xx7.setText(str(xx[6]))

        msg = "已切换到增量模式！\n"
        self.textEdit.setText(msg)

    def go_home(self):
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
        self.command_qq = np.copy(qq)
        msg = "运动到家点已规划！\n"
        self.textEdit.setText(msg)

    # 增量运动程序
    def inc_run(self):
        t = 5
        qq_command = np.zeros(self.n)
        xx2 = np.zeros(7)
        qq_list=0
        if (self.joint_flag):
            qq_command[0] = self.lineEdit_qq1.text()
            qq_command[1] = self.lineEdit_qq2.text()
            qq_command[2] = self.lineEdit_qq3.text()
            qq_command[3] = self.lineEdit_qq4.text()
            qq_command[4] = self.lineEdit_qq5.text()
            qq_command[5] = self.lineEdit_qq6.text()
            qq_command[6] = self.lineEdit_qq7.text()
            qq_command = qq_command * np.pi / 180.0

            # 判断是否超出关节极限
            flag = gf.out_joint_limit(qq_command, self.q_min, self.q_max)
            if (flag):
                msg = "超出关节极限！！！\n" + "请反向操作撤回！"
                self.textEdit.setText(msg)
                return -1

            t = int(self.dq) / 5 + 5
            # 更新末端位置
            xx = self.my_kin.fkine_zeros(qq_command)
            xx[:3] = xx[:3] * 1000
            xx[3:] = xx[3:] * 180.0 / np.pi
            xx = np.around(xx, 2)
            self.lineEdit_xx1.setText(str(xx[0]))
            self.lineEdit_xx2.setText(str(xx[1]))
            self.lineEdit_xx3.setText(str(xx[2]))
            self.lineEdit_xx4.setText(str(xx[3]))
            self.lineEdit_xx5.setText(str(xx[4]))
            self.lineEdit_xx6.setText(str(xx[5]))
            self.lineEdit_xx7.setText(str(xx[6]))
            [qq_list, qv, qa] = gf.q_joint_space_plan_time(self.qq_state, qq_command,
                                                           T=self.T, t=t)

        else:
            xx2[0] = self.lineEdit_xx1.text()
            xx2[1] = self.lineEdit_xx2.text()
            xx2[2] = self.lineEdit_xx3.text()
            xx2[3] = self.lineEdit_xx4.text()
            xx2[4] = self.lineEdit_xx5.text()
            xx2[5] = self.lineEdit_xx6.text()
            xx2[6] = self.lineEdit_xx7.text()
            xx2[:3] = xx2[:3] * 0.001
            xx2[3:] = xx2[3:] * np.pi / 180.0
            t = int(self.dx) / 5 + 5
            # 设计笛卡尔空间规划
            xx1 = self.my_kin.fkine_zeros(self.qq_state)
            self.line_plan.get_begin_end_point(xx1, xx2)
            self.line_plan.get_init_guess_joint(self.qq_state)

            [qq_list, qv, qa] = self.line_plan.out_joint()

            # 更新关节角度
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
        t1 = np.linspace(0, self.T * (num - 1), num)
        self.plot_vel(t1, qv)
        self.plot_pos(t1, qq_list)
        self.command_qq = np.copy(qq_list)

        if(self.joint_flag):
            self.begin_function()
        else:
            msg = "规划完成,请认真检查位置图是否有跳跃！\n若无误运行开始！"
            self.textEdit.setText(msg)

    # ---------增量按钮组---------#
    def fun_qq1_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq1.text())
        qq = qq - self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq1.setText(str(qq))
        # 启动程序
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
        # 启动程序
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
        # 启动程序
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
        # 启动程序
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
        # 启动程序
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
        # 启动程序
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
        # 启动程序
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

    def fun_xx7_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx7.text())
        xx = xx - self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx7.setText(str(xx))
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

    def fun_xx7_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx7.text())
        xx = xx + self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx7.setText(str(xx))
        # 启动程序
        self.inc_run()

    # ----------任务按钮---------#
    #门把手
    def handle_go_init(self):
        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        qq_h_init = np.array([-60.0, 65, 50.73, 93, -20, -44.31, 0])*np.pi/180.0
        t = 10
        # 调用规划函数
        [qq, qv, _] = gf.q_joint_space_plan_time(qq_b, qq_h_init, self.T, t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到门把手任务初始点已规划！\n"
        self.textEdit.setText(msg)

    def handle_run(self):
        if(not self.visual_flag):
            msg = "视觉检测失败,请重新检测再规划！\n"
            self.textEdit.setText(msg)
            return -1

        #获得规划时间
        t = 10
        # 调用规划函数
        #运行到门把手位置
        xx1_1 = self.my_kin.fkine_zeros(self.qq_state)
        xx1_2 = np.copy(xx1_1)
        xx1_2[0:3] = self.handle_X[0:3]

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx1_1, xx1_2)
        self.line_plan.get_init_guess_joint(self.qq_state)
        [qq1, qv1, qa1] = self.line_plan.out_joint()

        #转动门把手
        t=5
        qq2_b = qq1[-1, :]
        qq2_e = qq2_b + np.array([0, 0, 0, 0, 0, 0, 60])*np.pi/180.0
        [qq2_1, qv2_1, qa] = gf.q_joint_space_plan_time(qq2_b, qq2_e, self.T, t)
        kk2 = len(qq2_1)
        qq2 = np.zeros([2*kk2, self.n])
        qv2 = np.zeros([2*kk2, self.n])
        qq2[:kk2] = qq2_1
        qq2[kk2:] = qq2_1[::-1]
        qv2[:kk2] = qv2_1
        qv2[kk2:] = qv2_1[::-1]


        #拉门开门
        #xx2_1 = self.my_kin.fkine_zeros(qq2[-1, :])
        xx2_1 = xx1_2
        xx2_2 = np.copy(xx2_1)
        xx2_2[0] = xx2_2[0] - 0.02

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx2_1, xx2_2)
        self.line_plan.get_init_guess_joint(qq2[0, :])
        [qq3, qv3, qa] = self.line_plan.out_joint()

        #合成一个完整轨迹
        k1 = len(qq1)
        k2 = len(qq2)
        k3 = len(qq3)
        ks = 200
        k = k1 + k2 + k3 + 2*ks

        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[0:k1] = qq1
        qv[0:k1] = qv1
        qq[k1:k1+ks] = np.dot(np.ones([ks, self.n]), np.diag(qq1[-1, :]))
        qv[k1:k1+ks] = np.dot(np.ones([ks, self.n]), np.diag(qv1[-1, :]))
        qq[k1+ks:k1+ks+k2] = qq2
        qv[k1+ks:k1+ks+k2] = qv2
        qq[k1+ks+k2:k1+ks+k2+k3] = qq3
        qv[k1+ks+k2:k1+ks+k2+k3] = qv3
        qq[k1+ks+k2+k3:] = np.dot(np.ones([ks, self.n]), np.diag(qq2[-1, :]))
        qv[k1+ks+k2+k3:] = np.dot(np.ones([ks, self.n]), np.diag(qv2[-1, :]))

        # 调用绘图函数
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到门把手任务初始点已规划！\n"
        self.textEdit.setText(msg)

    def handle_back_init(self):
        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        qq_init = np.array([-60.0, 65, 50.73, 93, -20, -44.31, 0])*np.pi/180.0
        t = 10
        # 调用规划函数
        [qq, qv, _] = gf.q_joint_space_plan_time(qq_b, qq_init, self.T, t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    def lock_go_init(self):
        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        qq_h_init = np.array([-60.0, 65, 50.73, 93, -20, -44.31, 0])*np.pi/180.0
        t = 10
        # 调用规划函数
        [qq, qv, _] = gf.q_joint_space_plan_time(qq_b, qq_h_init, self.T, t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到门把手任务初始点已规划！\n"
        self.textEdit.setText(msg)

    def lock_run(self):
        if(not self.visual_flag):
            msg = "视觉检测失败,请重新检测再规划！\n"
            self.textEdit.setText(msg)
            return -1

        #获得规划时间
        t = 10
        # 调用规划函数
        #运行到门把手位置
        xx1_1 = self.my_kin.fkine_zeros(self.qq_state)
        xx1_2 = np.copy(xx1_1)
        xx1_2[0:3] = self.lock_X[0:3]

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx1_1, xx1_2)
        self.line_plan.get_init_guess_joint(self.qq_state)
        [qq1, qv1, qa1] = self.line_plan.out_joint()

        #转动门把手
        t=5
        qq2_b = qq1[-1, :]
        qq2_e = qq2_b + np.array([0, 0, 0, 0, 0, 0, 60])*np.pi/180.0
        [qq2, qv2, qa] = gf.q_joint_space_plan_time(qq2_b, qq2_e, self.T, t)

        #拉门开门
        xx2_1 = self.my_kin.fkine_zeros(qq2[-1, :])
        xx2_2 = np.copy(xx2_1)
        xx2_2[0] = xx2_2[0] - 0.05

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx2_1, xx2_2)
        self.line_plan.get_init_guess_joint(qq2[-1, :])
        [qq3, qv3, qa] = self.line_plan.out_joint()

        #合成一个完整轨迹
        k1 = len(qq1)
        k2 = len(qq2)
        k3 = len(qq3)
        ks = 200
        k = k1 + k2 + k3 + 2*ks

        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[0:k1] = qq1
        qv[0:k1] = qv1
        qq[k1:k1+ks] = np.dot(np.ones([ks, self.n]), np.diag(qq1[-1, :]))
        qv[k1:k1+ks] = np.dot(np.ones([ks, self.n]), np.diag(qv1[-1, :]))
        qq[k1+ks:k1+ks+k2] = qq2
        qv[k1+ks:k1+ks+k2] = qv2
        qq[k1+ks+k2:k1+ks+k2+ks] = np.dot(np.ones([ks, self.n]), np.diag(qq2[-1, :]))
        qv[k1+ks+k2:k1+ks+k2+ks] = np.dot(np.ones([ks, self.n]), np.diag(qv2[-1, :]))
        qq[k1+ks+k2+ks:] = qq3
        qv[k1+ks+k2+ks:] = qv3

        # 调用绘图函数
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到门把手任务初始点已规划！\n"
        self.textEdit.setText(msg)

    def lock_back_init(self):
        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        qq_init = np.array([-60.0, 65, 50.73, 93, -20, -44.31, 0])*np.pi/180.0
        t = 10
        # 调用规划函数
        [qq, qv, _] = gf.q_joint_space_plan_time(qq_b, qq_init, self.T, t)
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    #主函数运行
    def begin_function(self):
        # 运行标签启动
        self.run_flag = True
        self.run_finish = False

        # 提示标语
        msg = "开始下发命令！\n"
        self.textEdit.setText(msg)

        # 设置绘图,用Qtimer开线程处理（线程4）
        if (not self.real_flag):
            self.timer_plot = QTimer()
            self.timer_plot.timeout.connect(self.realtime_plot)
            self.timer_plot.start(1000)

        # 发送关节角度
        kk = len(self.command_qq)
        # # 进度条显示时间间隔
        show_time = int(kk * self.T * 10)

        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(show_time)

        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if ((not self.run_flag) or k == kk):
                # self.timer_p.stop()
                if (not self.real_flag):
                    self.timer_plot.stop()
                break

            command_data = Float64MultiArray()
            command_data.data = self.command_qq[k, 0:7]
            self.pub.publish(command_data)
            if (k % 20 == 0):
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

# ================armc窗口4:零空间与最小范数相结合================#
class ArmcWindow4(QMainWindow, Ui_ArmcForm4):
    # 建立全局变量
    state_qq_list = list(np.zeros([1000, 7]))
    state_f_list = list(np.zeros([1000, 6]))
    state_t_list = list(np.zeros(1000))
    state_t = 0

    def __init__(self, parent=None):
        super(ArmcWindow4, self).__init__(parent)
        self.T = 0.01
        self.t = 10

        self.run_flag = False  # 开始或停止标签
        self.run_finish = True
        self.real_flag = False
        self.joint_flag = False

        self.pos1 = False
        self.pos2 = False

        self.read_flag = False
        self.visual_flag = False

        self.flag_h = False
        self.flag_l = False
        self.flag_g = False

        [DH0, q_max, q_min] = gf.get_robot_parameter("armc")
        self.DH0 = DH0
        self.q_max = q_max
        self.q_min = q_min
        self.robot = "armc"
        # 创建运动学求解器
        self.my_kin = kin.GeneralKinematic(self.DH0, self.q_min, self.q_max)

        # 建立规划类
        self.line_plan = pap.ArmcLinePlan()
        self.line_plan.get_period(self.T)
        self.line_plan.get_robot_parameter(DH0, q_min, q_max)

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
        self.button_read.clicked.connect(self.read_qq)
        self.checkBox_real.stateChanged.connect(self.real_and_joint)
        self.button_get_init.clicked.connect(self.calculation_init_point_xx)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_init.clicked.connect(self.go_init)
        self.button_inc.clicked.connect(self.go_inc)
        self.button_home.clicked.connect(self.go_home)

        # -------增量控制按钮栏--------#
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
        self.button_xx7_N.clicked.connect(self.fun_xx7_n)
        self.button_xx1_P.clicked.connect(self.fun_xx1_p)
        self.button_xx2_P.clicked.connect(self.fun_xx2_p)
        self.button_xx3_P.clicked.connect(self.fun_xx3_p)
        self.button_xx4_P.clicked.connect(self.fun_xx4_p)
        self.button_xx5_P.clicked.connect(self.fun_xx5_p)
        self.button_xx6_P.clicked.connect(self.fun_xx6_p)
        self.button_xx7_P.clicked.connect(self.fun_xx7_p)

        self.button_check.clicked.connect(self.visual_check)
        self.button_h_go.clicked.connect(self.handle_go_init)
        self.button_h_run.clicked.connect(self.handle_run)
        self.button_h_back.clicked.connect(self.handle_back_init)
        self.button_l_go.clicked.connect(self.lock_go_init)
        self.button_l_run.clicked.connect(self.lock_run)
        self.button_l_back.clicked.connect(self.lock_back_init)
        self.button_g_go.clicked.connect(self.grab_go_init)
        self.button_g_run.clicked.connect(self.grab_run)
        self.button_g_back.clicked.connect(self.grab_back_init)

        self.button_qq_to_xx.clicked.connect(self.joint_to_twist)

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

    # 调用视觉检测
    def visual_check(self):
        # 读取检测目标种类
        self.flag_h = self.radioButton_handle.isChecked()
        self.flag_l = self.radioButton_lock.isChecked()
        self.flag_g = self.radioButton_grab.isChecked()
        a=0
        if(self.flag_h):
            a =1
        if(self.flag_l):
            a = 0
        if(self.flag_g):
            a = 2
        #print "a:" , a
        rospy.wait_for_service('visual_inspection')
        try:
            visual = rospy.ServiceProxy('visual_inspection', VisualVar)
            resp = visual(a)
            print "resp:", resp
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        # 位置是否检测成功
        self.visual_flag = resp.flag
        if (self.visual_flag):
            if (a == 1):
                handle_X = np.array(resp.T[0:6])
                R = self.my_kin.euler_zyx2rot(handle_X[3:6])
                Tc_o = np.eye(4)
                Tc_o[0:3, 0:3] = R
                Tc_o[0:3, 3] = handle_X[0:3]*0.001
                flag = False
                if(Tc_o[2, 3] < 0):
                    flag = True
                    msg_tip = "检测错误！"
                    #Tc_o[0:3, 3] = - Tc_o[0:3, 3]

                Te_c = gf.camera_parameters()
                T0_e = self.my_kin.fkine(self.qq_state)
                T0_o = np.dot(np.dot(T0_e, Te_c), Tc_o)
                handle_X[0:3] = T0_o[0:3, 3]
                handle_X[3:6] = self.my_kin.rot2euler_zyx(T0_o[0:3, 0:3])
                self.handle_X = handle_X

                handle_X1 = np.array(resp.T[6:12])
                R1 = self.my_kin.euler_zyx2rot(handle_X1[3:6])
                Tc_o1 = np.eye(4)
                Tc_o1[0:3, 0:3] = R1
                Tc_o1[0:3, 3] = handle_X1[0:3] * 0.001
                print "Tc_o1:", np.around(Tc_o1, 3)

                Te_c = gf.camera_parameters()
                T0_e1 = self.my_kin.fkine(self.qq_state)
                T0_o1 = np.dot(np.dot(T0_e1, Te_c), Tc_o1)

                handle_X1[0:3] = T0_o1[0:3, 3]
                handle_X1[3:6] = self.my_kin.rot2euler_zyx(T0_o1[0:3, 0:3])
                self.handle_X1 = handle_X1

                msg = "门把手检测成功！\n"
                msg1 = "第一组解：" + "\nx:" + str(handle_X[0]*1000) + "\ny:" + str(handle_X[1]*1000) \
                      + "\nz:" + str(handle_X[2]*1000) + "\nR:" + str(handle_X[3]) \
                      + "\nP:" + str(handle_X[4]) + "\nY:" + str(handle_X[5]) + "\n"
                msg2 = "第二组解：" + str(handle_X1[0]*1000) + "\ny:" + str(handle_X1[1]*1000) \
                      + "\nz:" + str(handle_X1[2]*1000) + "\nR:" + str(handle_X1[3]) \
                      + "\nP:" + str(handle_X1[4]) + "\nY:" + str(handle_X1[5]) + "\n"
                if(flag):
                    msg = msg_tip
                msg = msg + msg1 + msg2
            else:
                Tc_o = np.eye(4)
                for i in range(12):
                    Tc_o[i / 4, i% 4] = resp.T[i]

                Tc_o[0:3, 3] = Tc_o[0:3, 3]*0.001
                print "Tc_o:", Tc_o
                #Tc_o = np.linalg.inv(To_c)
                # Tc_o = np.eye(4)
                # Tc_o[0:3, 0:3] = To_c[0:3, 0:3].T
                # Tc_o[0:3, 3] = np.dot(-Tc_o[0:3, 0:3],  To_c[0:3, 3])
                # print " Tc_o:\n", np.around(Tc_o, 6)
                Te_c = gf.camera_parameters()
                T0_e = self.my_kin.fkine(self.qq_state)
                # print "T0_e:\n", np.around(T0_e, 2)
                # print "T0_c:\n", np.around(np.dot(T0_e, Te_c), 2)
                T0_o = np.dot(np.dot(T0_e, Te_c), Tc_o)
                # print "T0_o:\n", np.around(T0_o, 2)
                lock_X = np.zeros(6)
                lock_X[0:3] = T0_o[0:3, 3]
                lock_X[3:6] = self.my_kin.rot2euler_zyx(T0_o[0:3, 0:3])
                self.lock_X = lock_X
                msg = "密码锁检测成功！"
                msg = msg + "\nx:" + str(lock_X[0]*1000) + "\ny:" + str(lock_X[1]*1000) \
                      + "\nz:" + str(lock_X[2]*1000) + "\nR:" + str(lock_X[3]) \
                      + "\nP:" + str(lock_X[4]) + "\nY:" + str(lock_X[5])
        else:
            msg = "检测失败！！！"
        self.textEdit.setText(msg)

    def visual_check1(self):
        # 读取检测目标种类
        self.flag_h = self.radioButton_handle.isChecked()
        self.flag_l = self.radioButton_lock.isChecked()
        self.flag_g = self.radioButton_grab.isChecked()
        self.visual_flag = True
        self.handle_X = np.array([0.620, 0.136, 0.40, 0, 0, 0])
        self.lock_X = np.array([0.70, 0.025, 0.6, 0, 0, 0])
        self.grab_X = np.array([0.600, -0.11, 0.400, 0, 0, 0])
        msg = "检测成功！"
        self.textEdit.setText(msg)

    # 计算初始位置
    def calculation_init_point_xx(self):
        # 初始关节角
        flag1 = self.radioButton_handle.isChecked()
        flag2 = self.radioButton_lock.isChecked()
        flag3 = self.radioButton_grab.isChecked()
        if(flag1):
            qq = np.array([9, -50, 0, 70, 0, 90, -92])
        if (flag2):
            qq = np.array([-8, -50, 0, 70, 0, 85, -92])
        if(flag3):
            qq = np.array([-3, -50, 0, 70, 0, 90, -90])
        self.lineEdit_qq1.setText(str(qq[0]))
        self.lineEdit_qq2.setText(str(qq[1]))
        self.lineEdit_qq3.setText(str(qq[2]))
        self.lineEdit_qq4.setText(str(qq[3]))
        self.lineEdit_qq5.setText(str(qq[4]))
        self.lineEdit_qq6.setText(str(qq[5]))
        self.lineEdit_qq7.setText(str(qq[6]))

        qq[0] = self.lineEdit_qq1.text()
        qq[1] = self.lineEdit_qq2.text()
        qq[2] = self.lineEdit_qq3.text()
        qq[3] = self.lineEdit_qq4.text()
        qq[4] = self.lineEdit_qq5.text()
        qq[5] = self.lineEdit_qq6.text()
        qq[6] = self.lineEdit_qq7.text()

        # 转换单位
        qq = qq * np.pi / 180.0

        # 计算初始位置
        xx_b = self.my_kin.fkine_zeros(qq)

        # 转换到显示单位
        xx = np.copy(xx_b)  # 转化为mm显示
        xx[0:3] = xx_b[0:3] * 1000  # 转换为mm显示
        xx[3:] = xx_b[3:] * 180.0 / np.pi  # 转换为mm显示

        # 显示到界面
        self.lineEdit_xx1.setText(str(round(xx[0], 4)))
        self.lineEdit_xx2.setText(str(round(xx[1], 4)))
        self.lineEdit_xx3.setText(str(round(xx[2], 4)))
        self.lineEdit_xx4.setText(str(round(xx[3], 4)))
        self.lineEdit_xx5.setText(str(round(xx[4], 4)))
        self.lineEdit_xx6.setText(str(round(xx[5], 4)))
        self.lineEdit_xx7.setText(str(round(xx[6], 4)))
        msg = "初始末端位置\n" + \
              "x1:" + str(xx[0]) + "\n" + "x2:" + str(xx[1]) + \
              "\n" + "x3:" + str(xx[2]) + "\n" + "x4:" + str(xx[3]) + \
              "\n" + "x5:" + str(xx[4]) + "\n" + "x6:" + str(xx[5]) + "\n" \
              + "psi:" + str(xx[6]) + "\n"
        self.textEdit.setText(msg)

    def read_qq(self):
        # 初始关节角
        qq = np.zeros(7)
        qq[0] = self.lineEdit_qq1.text()
        qq[1] = self.lineEdit_qq2.text()
        qq[2] = self.lineEdit_qq3.text()
        qq[3] = self.lineEdit_qq4.text()
        qq[4] = self.lineEdit_qq5.text()
        qq[5] = self.lineEdit_qq6.text()
        qq[6] = self.lineEdit_qq7.text()

        # 转换单位
        qq1 = qq * np.pi / 180.0
        self.qq_init = np.copy(qq1)

        msg = "读取目标关节角:\n" + \
              "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + "\n" + \
              "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + "\n" + \
              "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + "\n" + \
              "q7:" + str(qq[6]) + "\n"
        self.textEdit.setText(msg)

        self.read_flag = True

    #关节空间到笛卡尔空间
    def joint_to_twist(self):
        qq = np.array([-40, 23, 15, 25, 60, 80, -100])
        self.lineEdit_qq1.setText(str(qq[0]))
        self.lineEdit_qq2.setText(str(qq[1]))
        self.lineEdit_qq3.setText(str(qq[2]))
        self.lineEdit_qq4.setText(str(qq[3]))
        self.lineEdit_qq5.setText(str(qq[4]))
        self.lineEdit_qq6.setText(str(qq[5]))
        self.lineEdit_qq7.setText(str(qq[6]))
        # 初始关节角
        qq = np.zeros(self.n)
        qq[0] = self.lineEdit_qq1.text()
        qq[1] = self.lineEdit_qq2.text()
        qq[2] = self.lineEdit_qq3.text()
        qq[3] = self.lineEdit_qq4.text()
        qq[4] = self.lineEdit_qq5.text()
        qq[5] = self.lineEdit_qq6.text()
        qq[6] = self.lineEdit_qq7.text()

        # 转换单位
        qq = qq * np.pi / 180.0

        # 计算初始位置
        xx_b = self.my_kin.fkine_zeros(qq)

        # 转换到显示单位
        xx = np.copy(xx_b)  # 转化为mm显示
        xx[0:3] = xx_b[0:3] * 1000  # 转换为mm显示
        xx[3:] = xx_b[3:] * 180.0 / np.pi  # 转换为mm显示

        # 显示到界面
        self.lineEdit_xx1.setText(str(round(xx[0], 4)))
        self.lineEdit_xx2.setText(str(round(xx[1], 4)))
        self.lineEdit_xx3.setText(str(round(xx[2], 4)))
        self.lineEdit_xx4.setText(str(round(xx[3], 4)))
        self.lineEdit_xx5.setText(str(round(xx[4], 4)))
        self.lineEdit_xx6.setText(str(round(xx[5], 4)))
        self.lineEdit_xx7.setText(str(round(xx[6], 4)))
        msg = "初始末端位置\n" + \
              "x1:" + str(xx[0]) + "\n" + "x2:" + str(xx[1]) + \
              "\n" + "x3:" + str(xx[2]) + "\n" + "x4:" + str(xx[3]) + \
              "\n" + "x5:" + str(xx[4]) + "\n" + "x6:" + str(xx[5]) + "\n" \
              + "psi:" + str(xx[6]) + "\n"
        self.textEdit.setText(msg)

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

        # 获得规划起点
        qq_b = self.qq_state
        qq_m = np.copy(qq_b)
        qq_m[5] = self.qq_init[5]
        # 调用规划函数
        [qq1, qv1, _] = gf.q_joint_space_plan_time(qq_b, qq_m, self.T, self.t)
        [qq2, qv2, _] = gf.q_joint_space_plan_time(qq1[-1, :], self.qq_init, self.T, self.t)
        k1 = len(qq1)
        k2 = len(qq2)
        k = k1+k2
        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[:k1, :] = qq1
        qv[:k1, :] = qv1
        qq[k1:, :] = qq2
        qv[k1:, :] = qv2
        # 调用绘图函数
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    def go_inc(self):
        xx = self.my_kin.fkine_zeros(self.qq_state)
        self.XX = np.copy(xx)
        self.qq = np.copy(self.qq_state)
        qq = self.qq_state * 180 / np.pi
        qq = np.around(qq, 2)
        xx[:3] = xx[:3] * 1000
        xx[3:] = xx[3:] * 180 / np.pi
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
        self.lineEdit_xx7.setText(str(xx[6]))

        msg = "已切换到增量模式！\n"
        self.textEdit.setText(msg)

    def go_home(self):
        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        qq_home = np.array([0, 0, 0, 0, 0, 0, 0.0])
        # 调用规划函数
        qq_m = np.copy(qq_b) #设置中转点
        qq_m[1] = 0.0
        qq_m[3] = 0.0
        [qq1, qv1, _] = gf.q_joint_space_plan_time(qq_b, qq_m, self.T, self.t)
        [qq2, qv2, _] = gf.q_joint_space_plan_time(qq1[-1, :], qq_home, self.T, self.t)
        k1 = len(qq1)
        k2 = len(qq2)
        k = k1 + k2
        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[:k1, :] = qq1
        qv[:k1, :] = qv1
        qq[k1:, :] = qq2
        qv[k1:, :] = qv2
        # 调用绘图函数
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_vel(t, qv)
        self.plot_pos(t, qq)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到家点已规划！\n"
        self.textEdit.setText(msg)

    # 增量运动程序
    def inc_run(self):
        t = 5
        qq_command = np.zeros(self.n)
        xx2 = np.zeros(7)
        qq_list = 0
        if (self.joint_flag):
            qq_command[0] = self.lineEdit_qq1.text()
            qq_command[1] = self.lineEdit_qq2.text()
            qq_command[2] = self.lineEdit_qq3.text()
            qq_command[3] = self.lineEdit_qq4.text()
            qq_command[4] = self.lineEdit_qq5.text()
            qq_command[5] = self.lineEdit_qq6.text()
            qq_command[6] = self.lineEdit_qq7.text()
            qq_command = qq_command * np.pi / 180.0

            # 判断是否超出关节极限
            flag = gf.out_joint_limit(qq_command, self.q_min, self.q_max)
            if (flag):
                msg = "超出关节极限！！！\n" + "请反向操作撤回！"
                self.textEdit.setText(msg)
                return -1

            t = int(self.dq) / 5 + 5
            # 更新末端位置
            xx = self.my_kin.fkine_zeros(qq_command)
            xx[:3] = xx[:3] * 1000
            xx[3:] = xx[3:] * 180.0 / np.pi
            xx = np.around(xx, 2)
            self.lineEdit_xx1.setText(str(xx[0]))
            self.lineEdit_xx2.setText(str(xx[1]))
            self.lineEdit_xx3.setText(str(xx[2]))
            self.lineEdit_xx4.setText(str(xx[3]))
            self.lineEdit_xx5.setText(str(xx[4]))
            self.lineEdit_xx6.setText(str(xx[5]))
            self.lineEdit_xx7.setText(str(xx[6]))
            [qq_list, qv, qa] = gf.q_joint_space_plan_time(self.qq_state, qq_command,
                                                           T=self.T, t=t)

        else:
            xx2[0] = self.lineEdit_xx1.text()
            xx2[1] = self.lineEdit_xx2.text()
            xx2[2] = self.lineEdit_xx3.text()
            xx2[3] = self.lineEdit_xx4.text()
            xx2[4] = self.lineEdit_xx5.text()
            xx2[5] = self.lineEdit_xx6.text()
            xx2[6] = self.lineEdit_xx7.text()
            xx2[:3] = xx2[:3] * 0.001
            xx2[3:] = xx2[3:] * np.pi / 180.0
            t = int(self.dx) / 5 + 5
            # 设计笛卡尔空间规划
            xx1 = self.my_kin.fkine_zeros(self.qq_state)
            self.line_plan.get_begin_end_point(xx1[:6], xx2[:6])
            self.line_plan.get_init_guess_joint(self.qq_state)

            [qq2, qv2, _] = self.line_plan.out_joint()

            t = 2
            [qq1, qv1, _] = gf.q_joint_space_plan_time(self.qq_state, qq2[0, :],
                                                             T=self.T, t=t)
            k1 = len(qq1)
            k2 = len(qq2)
            k = k1 + k2
            qq_list = np.zeros([k, self.n])
            qv = np.zeros([k, self.n])
            qq_list[:k1, :] = qq1
            qv[:k1, :] = qv1
            qq_list[k1:, :] = qq2
            qv[k1:, :] = qv2
            # 更新关节角度
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
        t1 = np.linspace(0, self.T * (num - 1), num)
        self.plot_vel(t1, qv)
        self.plot_pos(t1, qq_list)
        self.command_qq = np.copy(qq_list)

        if (self.joint_flag):
            self.begin_function()
        else:
            msg = "规划完成,请认真检查位置图是否有跳跃！\n若无误运行开始！"
            self.textEdit.setText(msg)

    # ---------增量按钮组---------#
    def fun_qq1_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = True
        self.dq = float(self.lineEdit_dq.text())
        qq = float(self.lineEdit_qq1.text())
        qq = qq - self.dq
        qq = np.round(qq, 2)
        self.lineEdit_qq1.setText(str(qq))
        # 启动程序
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
        # 启动程序
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
        # 启动程序
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
        # 启动程序
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
        # 启动程序
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
        # 启动程序
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
        # 启动程序
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

    def fun_xx7_n(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx7.text())
        xx = xx - self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx7.setText(str(xx))
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

    def fun_xx7_p(self):
        if (not self.run_finish):
            return -1
        self.joint_flag = False
        self.dx = float(self.lineEdit_dx.text())
        xx = float(self.lineEdit_xx7.text())
        xx = xx + self.dx
        xx = np.round(xx, 2)
        self.lineEdit_xx7.setText(str(xx))
        # 启动程序
        self.inc_run()

    # ----------任务按钮---------#
    # 门把手
    def handle_go_init(self):
        if(not self.flag_h):
            msg = "请先选着门把手！\n"
            self.textEdit.setText(msg)
            return 0
        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        #qq_h_init = np.array([-102.6, 32.7, 108.2, 71.6, -3.3, 69.9, 3.1]) * np.pi / 180.0
        #qq_h_init = np.array([-84.7, -14.1, 27.2, 87.5, 59.5, 78.9, -36.4]) * np.pi / 180.0
        #qq_h_init = np.array([-53.3, -18.7, 25.5, 92.3, 61.9, 76.5, -5.9]) * np.pi / 180.0
        qq_h_init = np.array([-56.3, -14.0, 32.9, 90.5, 59.3, 65.5, -3.6]) * np.pi / 180.0
        self.qq_h_init = qq_h_init
        t = 15
        # 调用规划函数
        qq_m = np.copy(qq_b)
        qq_m[0] = qq_h_init[0]
        qq_m[3] = qq_h_init[3]
        #qq_m[4] = qq_h_init[4]
        #qq_m[5] = qq_h_init[5]
        qq_m[6] = qq_h_init[6]
        [qq1, qv1, _] = gf.q_joint_space_plan_time(qq_b, qq_m, self.T, t)
        t = 15
        [qq2, qv2, _] = gf.q_joint_space_plan_time(qq1[-1, :], qq_h_init, self.T, t)
        k1 = len(qq1)
        k2 = len(qq2)
        k = k1 + k2
        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[:k1, :] = qq1
        qv[:k1, :] = qv1
        qq[k1:, :] = qq2
        qv[k1:, :] = qv2
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到门把手任务初始点已规划！\n"
        self.textEdit.setText(msg)

    def handle_run(self):
        if (not self.flag_h):
            msg = "请先选着门把手！\n"
            self.textEdit.setText(msg)
            return 0
        if (not self.visual_flag):
            msg = "视觉检测失败,请重新检测再规划！\n"
            self.textEdit.setText(msg)
            return -1

        # 获得规划时间
        t = 10
        # 调用规划函数
        # 运行到门把手位置
        xx1_1 = self.my_kin.fkine_zeros(self.qq_state)
        xx1_2 = np.copy(xx1_1)
        xx1_2[0:3] = self.handle_X[0:3]
        xx1_2[1] = self.handle_X[1] - 0.03

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx1_1[:6], xx1_2[:6])
        self.line_plan.get_init_guess_joint(self.qq_state)
        [qq1, qv1, _] = self.line_plan.out_joint()

        # 转动门把手
        t = 5
        xx2 = np.copy(xx1_2)
        xx2[2] = xx2[2] - 0.05

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx1_2[:6], xx2[:6])
        self.line_plan.get_init_guess_joint(qq1[-1, :])
        [qq2_1, qv2_1, _] = self.line_plan.out_joint()
        kk2_1 = len(qq2_1)
        qq2 = np.zeros([2*kk2_1, self.n])
        qv2 = np.zeros([2*kk2_1, self.n])
        qq2[:kk2_1] = qq2_1
        qv2[:kk2_1] = qv2_1
        qq2[kk2_1:] = qq2_1[::-1]
        qv2[kk2_1:] = qv2_1[::-1]

        # 拉门开门
        xx3_1 = self.my_kin.fkine_zeros(qq2[-1, :])
        xx3_2 = np.copy(xx3_1)
        xx3_2[0] = xx3_2[0] - 0.08

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx3_1[:6], xx3_2[:6])
        self.line_plan.get_init_guess_joint(qq2[-1, :])
        [qq4, qv4, _] = self.line_plan.out_joint()

        #臂形调整
        [qq3, qv3, _] = gf.q_joint_space_plan_time(qq2[-1, :], qq4[0,:], self.T, t)

        # 合成一个完整轨迹
        k1 = len(qq1)
        k2 = len(qq2)
        k3 = len(qq3)
        k4 = len(qq4)
        ks = 500
        k = k1 + k2 + k3 + k4 + 2 * ks

        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[0:k1] = qq1
        qv[0:k1] = qv1
        qq[k1:k1 + ks] = np.dot(np.ones([ks, self.n]), np.diag(qq1[-1, :]))
        qv[k1:k1 + ks] = np.dot(np.ones([ks, self.n]), np.diag(qv1[-1, :]))
        qq[k1 + ks:k1 + ks + k2] = qq2
        qv[k1 + ks:k1 + ks + k2] = qv2
        qq[k1 + ks + k2:k1 + ks + k2 + k3] = qq3
        qv[k1 + ks + k2:k1 + ks + k2 + k3] = qv3
        qq[k1 + ks + k2 + k3:k1 + ks + k2 + k3 + k4] = qq4
        qv[k1 + ks + k2 + k3:k1 + ks + k2 + k3 + k4] = qv4
        qq[k1 + ks + k2 + k3 + k4:] = np.dot(np.ones([ks, self.n]), np.diag(qq4[-1, :]))
        qv[k1 + ks + k2 + k3 + k4:] = np.dot(np.ones([ks, self.n]), np.diag(qv4[-1, :]))

        # 调用绘图函数
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到门把手任务初始点已规划！\n"
        self.textEdit.setText(msg)

    def handle_back_init(self):
        if (not self.flag_h):
            msg = "请先选着门把手！\n"
            self.textEdit.setText(msg)
            return 0
        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        qq_m1 = np.copy(self.qq_h_init)
        qq_init = np.array([-90, -45, 0, 90, 0, 90, 0]) * np.pi / 180.0
        t = 5
        # 调用规划函数
        [qq1, qv1, _] = gf.q_joint_space_plan_time(qq_b, qq_m1, self.T, t)

        qq_m2 = np.copy(qq1[-1, :])
        #qq_m2[0] = qq_init[0]
        qq_m2[1] = qq_init[1]
        qq_m2[2] = qq_init[2]
        qq_m2[4] = qq_init[4]
        #qq_m2[6] = qq_init[6]
        # 调用规划函数
        t = 15
        [qq2, qv2, _] = gf.q_joint_space_plan_time(qq1[-1, :], qq_m2, self.T, t)
        t = 15
        [qq3, qv3, _] = gf.q_joint_space_plan_time(qq2[-1, :], qq_init, self.T, t)
        k1 = len(qq1)
        k2 = len(qq2)
        k3 = len(qq3)
        k = k1 + k2 + k3
        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[:k1, :] = qq1
        qv[:k1, :] = qv1
        qq[k1:k1+k2, :] = qq2
        qv[k1:k1+k2, :] = qv2
        qq[k1+k2:, :] = qq3
        qv[k1+k2:, :] = qv3
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    #密码手
    def lock_go_init(self):
        if (not self.flag_l):
            msg = "请先选择密码锁！\n"
            self.textEdit.setText(msg)
            return 0
        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        #qq_h_init = np.array([-145.0, 39, 139.0, 91.7, 17.1, 39.3, 4.2]) * np.pi / 180.0
        qq_h_init = np.array([-39.5, -24.8, 17.4, 77.3, 25.1, 51.5, -0]) * np.pi / 180.0
        self.qq_l_init = np.copy(qq_h_init)
        t = 15
        # 调用规划函数
        qq_m = np.copy(qq_b)
        qq_m[0] = qq_h_init[0]
        qq_m[3] = qq_h_init[3]
        #qq_m[5] = qq_h_init[5]
        qq_m[6] = qq_h_init[6]
        [qq1, qv1, _] = gf.q_joint_space_plan_time(qq_b, qq_m, self.T, t)
        t = 15
        [qq2, qv2, _] = gf.q_joint_space_plan_time(qq1[-1, :], qq_h_init, self.T, t)
        k1 = len(qq1)
        k2 = len(qq2)
        k = k1 + k2
        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[:k1, :] = qq1
        qv[:k1, :] = qv1
        qq[k1:, :] = qq2
        qv[k1:, :] = qv2
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到密码锁任务初始点已规划！\n"
        self.textEdit.setText(msg)

    def lock_run(self):
        if (not self.flag_l):
            msg = "请先选择密码锁！\n"
            self.textEdit.setText(msg)
            return 0
        if (not self.visual_flag):
            msg = "视觉检测失败,请重新检测再规划！\n"
            self.textEdit.setText(msg)
            return -1

        # 获得规划时间
        t = 10
        # 调用规划函数
        # 运行到门把手位置
        xx1 = self.my_kin.fkine_zeros(self.qq_state)
        xx2 = np.copy(xx1)
        xx2[0:3] = self.lock_X[0:3]
        xx2[1] = xx2[1] - 0.02

        xx_m = np.copy(xx2)
        xx_m[0] = xx_m[0] - 0.05

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx1[:6], xx2[:6])
        self.line_plan.get_init_guess_joint(self.qq_state)
        [qq1, qv1, _] = self.line_plan.out_joint()

        #返回
        xx3 = np.copy(xx2)
        xx4 = np.copy(xx2)
        xx3[1] = xx3[1] + 0.01
        xx4[2] = xx4[2] - 0.01

        t=3
        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx2[:6], xx_m[0:6])
        self.line_plan.get_init_guess_joint(qq1[-1, :])
        [qq2_1, qv2_1, _] = self.line_plan.out_joint()

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx_m[:6], xx3[:6])
        self.line_plan.get_init_guess_joint(qq2_1[-1, :])
        [qq2_2, qv2_2, _] = self.line_plan.out_joint()

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx3[:6], xx_m[:6])
        self.line_plan.get_init_guess_joint(qq2_2[-1, :])
        [qq3_1, qv3_1, _] = self.line_plan.out_joint()

        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx_m[:6], xx4[:6])
        self.line_plan.get_init_guess_joint(qq3_1[-1, :])
        [qq3_2, qv3_2, _] = self.line_plan.out_joint()

        xx_m[0] = xx_m[0] - 0.06
        t = 10
        self.line_plan.get_plan_time(t)
        self.line_plan.get_begin_end_point(xx4[:6], xx_m[:6])
        self.line_plan.get_init_guess_joint(qq3_2[-1, :])
        [qq4, qv4, _] = self.line_plan.out_joint()

        # 合成一个完整轨迹
        k1 = len(qq1)
        k2 = len(qq2_1)
        k3 = len(qq2_2)
        k4 = len(qq3_1)
        k5 = len(qq3_2)
        k6 = len(qq4)

        k = k1 + k2 + k3 + k4 + k5 + k6

        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[0:k1] = qq1
        qv[0:k1] = qv1
        qq[k1:k1 + k2] = qq2_1
        qv[k1:k1 + k2] = qv2_1
        qq[k1 + k2:k1 + k2 + k3] = qq2_2
        qv[k1 + k2:k1 + k2 + k3] = qv2_2
        qq[k1 + k2 + k3:k1 + k2 + k3 + k4] = qq3_1
        qv[k1 + k2 + k3:k1 + k2 + k3 + k4] = qv3_1
        qq[k1 + k2 + k3 + k4:k1 + k2 + k3 + k4 + k5] = qq3_2
        qv[k1 + k2 + k3 + k4:k1 + k2 + k3 + k4 + k5] = qv3_2
        qq[k1 + k2 + k3 + k4 + k5:] = qq4
        qv[k1 + k2 + k3 + k4 + k5:] = qv4

        # 调用绘图函数
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到门把手任务初始点已规划！\n"
        self.textEdit.setText(msg)

    def lock_back_init(self):
        if (not self.flag_l):
            msg = "请先选择密码锁！\n"
            self.textEdit.setText(msg)
            return 0
        # 获得规划起点
        qq_b = np.array(self.state_qq_list[-1])
        qq_m1 = self.qq_l_init
        qq_init = np.array([-90, -45, 0, 90, 0, 90, 0]) * np.pi / 180.0
        t = 15
        # 调用规划函数
        [qq1, qv1, _] = gf.q_joint_space_plan_time(qq_b, qq_m1, self.T, t)

        qq_m2 = np.copy(qq1[-1, :])
        # qq_m2[0] = qq_init[0]
        qq_m2[1] = qq_init[1]
        qq_m2[2] = qq_init[2]
        qq_m2[4] = qq_init[4]
        qq_m2[5] = qq_init[5]
        # 调用规划函数
        t = 10
        [qq2, qv2, _] = gf.q_joint_space_plan_time(qq1[-1, :], qq_m2, self.T, t)
        t = 5
        [qq3, qv3, _] = gf.q_joint_space_plan_time(qq2[-1, :], qq_init, self.T, t)
        k1 = len(qq1)
        k2 = len(qq2)
        k3 = len(qq3)
        k = k1 + k2 + k3
        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[:k1, :] = qq1
        qv[:k1, :] = qv1
        qq[k1:k1 + k2, :] = qq2
        qv[k1:k1 + k2, :] = qv2
        qq[k1 + k2:, :] = qq3
        qv[k1 + k2:, :] = qv3
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    #抓取物体规划
    def grab_go_init(self):
        # 获得规划起点
        qq_b = self.qq_state
        qq_h_init = np.array([-16.5, 27.0, 15.0, 25.0, 0.0, 30.0, -100]) * np.pi / 180.0
        t = 15
        # 调用规划函数
        [qq, qv, _] = gf.q_joint_space_plan_time(qq_b, qq_h_init, self.T, t)
        t = 15
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到门把手任务初始点已规划！\n"
        self.textEdit.setText(msg)

    #用作开门
    def grab_run(self):
        # 获得规划时间
        t = 20
        #运动到指定位置
        qq_b = np.copy(self.qq_state)
        qq_m = np.copy(self.qq_state)
        qq_m[4] = qq_m[4] + np.pi/6
        qq_m[0] = qq_m[0] + np.pi / 6
        [qq, qv, _] = gf.q_joint_space_plan_time(qq_b, qq_m, self.T, t)

        k = len(qq)
        # 调用绘图函数
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到门把手任务初始点已规划！\n"
        self.textEdit.setText(msg)

    def grab_back_init(self):
        # 获得规划起点
        qq_b = self.qq_state
        qq_m = np.array([-45, 30, 0, 0, 0, 60, -90]) * np.pi / 180.0
        qq_init = np.array([-90, -45, 0, 90, 0, 90, 0]) * np.pi / 180.0

        # 调用规划函数
        t = 20
        [qq1, qv1, _] = gf.q_joint_space_plan_time(qq_b, qq_m, self.T, t)
        t = 20
        [qq2, qv2, _] = gf.q_joint_space_plan_time(qq_m, qq_init, self.T, t)
        k1 = len(qq1)
        k2 = len(qq2)
        ks = 500
        k = k1 + k2 + ks
        qq = np.zeros([k, self.n])
        qv = np.zeros([k, self.n])
        qq[:k1, :] = qq1
        qv[:k1, :] = qv1
        qq[k1:k1 + ks, :] = np.dot(np.ones([ks, self.n]), np.diag(qq1[-1, :]))
        qv[k1:k1 + ks, :] = np.dot(np.ones([ks, self.n]), np.diag(qv1[-1, :]))
        qq[k1 + ks:, :] = qq2
        qv[k1 + ks:, :] = qv2
        # 调用绘图函数
        k = len(qq[:, 0])
        t = np.linspace(0, self.T * (k - 1), k)
        # 绘制关节角位置速度图
        self.plot_pos(t, qq)
        self.plot_vel(t, qv)
        # 将规划好的位置定义为全局变量
        self.command_qq = np.copy(qq)
        msg = "运动到初始点已规划！\n"
        self.textEdit.setText(msg)

    # 主函数运行
    def begin_function(self):
        # 运行标签启动
        self.run_flag = True
        self.run_finish = False

        # 提示标语
        msg = "开始下发命令！\n"
        self.textEdit.setText(msg)

        # 设置绘图,用Qtimer开线程处理（线程4）
        if (not self.real_flag):
            self.timer_plot = QTimer()
            self.timer_plot.timeout.connect(self.realtime_plot)
            self.timer_plot.start(1000)

        # 发送关节角度
        kk = len(self.command_qq)
        # # 进度条显示时间间隔
        show_time = int(kk * self.T * 10)

        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(show_time)

        rate = rospy.Rate(100)
        k = 0
        while not rospy.is_shutdown():
            # 检测是否启动急停
            if ((not self.run_flag) or k == kk):
                # self.timer_p.stop()
                if (not self.real_flag):
                    self.timer_plot.stop()
                break

            command_data = Float64MultiArray()
            command_data.data = self.command_qq[k, 0:7]
            self.pub.publish(command_data)
            if (k % 20 == 0):
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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MainWindow()
    myWin.show()
    sys.exit(app.exec_())