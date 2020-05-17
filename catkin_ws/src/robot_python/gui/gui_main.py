#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于GUI开发界面函数
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2019.4.21
#系统函数
import sys
import numpy as np

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
from impedance_form1 import Ui_ImpForm1
from impedance_form2 import Ui_ImpForm2
from circularPlan_form1 import Ui_CirForm1
from linePlan_form1 import Ui_LineForm1
from rundata_form import Ui_RunForm
from test_form1 import Ui_TestForm1
from teaching_form1 import Ui_TeachingForm1


#自定义文件
import gui_function as gf
import gui_image as gi
from robot_python import ImpedanceControl as imp
from robot_python import TeachingLearning as tl
from robot_python import FileOpen as fo


#**********************************主窗口***************************************#
class MainWindow(QMainWindow, Ui_MainWindow):
    #建立全局变量
    state_qq_list = list(np.zeros([1000,7]))
    state_qv_list = list(np.zeros([1000,7]))
    state_t_list = list(np.zeros(1000))
    state_t = 0
    T = 0.01
    flag = 1 #开始或停止标签
    arm_flag = 0
    gazebo_flag = 0
    sub_path = "/robot3/joint_states"
    pub_path = "/robot3/armc_position_controller/command"
    n = 7  #机械臂关节数
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.initUI()
    def initUI(self):
        # ======================菜单栏功能模块=======================#
        #创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        planMenu = menubar.addMenu('&Plan')
        impedanceMenu = menubar.addMenu('Impedance')
        teachMenu = menubar.addMenu('&Teaching')
        demoMenu = menubar.addMenu('&Demo')
        testMenu = menubar.addMenu('&Test')
        helpMenu = menubar.addMenu('&Help')

        #文件菜单中打开文件操作
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
        #读取积分自适应阻抗参数MBKI
        self.Button_begin.clicked.connect(self.begin_function)
        self.button_end.clicked.connect(self.off)
        self.button_plan_begin.clicked.connect(self.read_joint_pos1)
        self.button_plan_end.clicked.connect(self.read_joint_pos2)
        self.button_plan.clicked.connect(self.plan)
        self.checkBox_UR5.stateChanged.connect(self.armc_or_ur5)

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
        self.horizontalLayout_pos.addWidget(win1)
        self.horizontalLayout_vel.addWidget(win2)

        p1 = win1.addPlot(title="joint pos")  # 添加第一个绘图窗口
        p1.setLabel('left', text='pos/rad', color='#ffffff')  # y轴设置函数
        p1.showGrid(x=True, y=True)  # 栅格设置函数
        p1.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p1.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p1.addLegend(size=(50, 30))  # 可选择是否添加legend

        p2 = win2.addPlot(title="joint vel")  # 添加第一个绘图窗口
        p2.setLabel('left', text='vel:rad/s', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1,p2

    #绘画关节角和关节角速度曲线
    def plot_joint(self,t,qq,qv):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t, qq[:, 6], pen='w', name='qq7', clear=False)
        # 绘制速度图
        self.p2.plot(t, qv[:, 0], pen='b', name='qv1', clear=True)
        self.p2.plot(t, qv[:, 1], pen='g', name='qv2', clear=False)
        self.p2.plot(t, qv[:, 2], pen='r', name='qv3', clear=False)
        self.p2.plot(t, qv[:, 3], pen='c', name='qv4', clear=False)
        self.p2.plot(t, qv[:, 4], pen='m', name='qv5', clear=False)
        self.p2.plot(t, qv[:, 5], pen='y', name='qv6', clear=False)
        self.p2.plot(t, qv[:, 6], pen='w', name='qv7', clear=False)

    #读取给定关节角度
    def read_joint_pos1(self):
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()
        msg_pos = "规划起始点\n" + \
                   "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) +\
                   "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) +\
                   "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                   "\n" + "q7:" + str(qq[6])
        self.textEdit.setText(msg_pos)
        self.qq_b = np.copy(qq*np.pi/180.0)

    def read_joint_pos2(self):
        qq = np.zeros(7)
        qq[0] = self.lineEdit_q1.text()
        qq[1] = self.lineEdit_q2.text()
        qq[2] = self.lineEdit_q3.text()
        qq[3] = self.lineEdit_q4.text()
        qq[4] = self.lineEdit_q5.text()
        qq[5] = self.lineEdit_q6.text()
        qq[6] = self.lineEdit_q7.text()
        msg_pos = "规划终止点\n" + \
                  "q1:" + str(qq[0]) + "\n" + "q2:" + str(qq[1]) + \
                  "\n" + "q3:" + str(qq[2]) + "\n" + "q4:" + str(qq[3]) + \
                  "\n" + "q5:" + str(qq[4]) + "\n" + "q6:" + str(qq[5]) + \
                  "\n" + "q7:" + str(qq[6])
        self.textEdit.setText(msg_pos)
        self.qq_e = np.copy(qq*np.pi/180.0)

    #规划函数
    def plan(self):
        #调用规划函数
        [qq,qv,T] = gf.q_joint_space_plan(self.qq_b, self.qq_e)
        #调用绘图函数
        k =len(qq)
        t = np.linspace(0, T*(k-1), k)
        self.T = T
        #绘制关节角位置速度图
        self.plot_joint(t, qq, qv)
        #将规划好的位置定义为全局变量
        self.qq_list = np.copy(qq)

    ##关节角订阅回调函数
    def joint_callback(self,msg):
        qq = np.zeros(self.n)
        qv = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]
            #qv[i] = msg.velocity[i]

        # 数据预处理
        qq_list = np.array(self.state_qq_list[-4:-1])
        qq_p = gf.joint_pos_Pretreatment(qq, qq_list, self.T)
        #qq_p = qq
        self.state_t = self.state_t + self.T
        self.state_qq_list.append(qq_p)
        self.state_qv_list.append(qv)
        self.state_t_list.append(self.state_t)
        # 仅记录100个数据点
        del self.state_t_list[0]
        del self.state_qq_list[0]
        del self.state_qv_list[0]

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def lcd_show(self):
        self.step_l = self.step_l + 1
        self.lcdNumber.display(self.step_l)

    def probar_show(self):
        self.step_p = self.step_p + 1
        self.progressBar.setValue(self.step_p)
        if(self.step_p>99):
            self.timer_p.stop()

    def off(self):
        self.flag = 0

    def realtime_plot(self):
        plot_t = np.array(self.state_t_list)
        plot_qq = np.array(self.state_qq_list)
        plot_qv = np.array(self.state_qv_list)
        self.plot_joint(plot_t, plot_qq, plot_qv)

    def armc_or_ur5(self):
        #运行该函数切换到ur5
        self.sub_path = "/robot1/joint_states"
        self.pub_path = "/robot1/ur5_position_controller/command"
        self.arm_flag = 1
        self.n = 6

    def begin_function(self):
        #运行标签启动
        self.flag = 1

        #运行话题
        rospy.init_node('mainWindon_run')
        rospy.Subscriber(self.sub_path, JointState, self.joint_callback)
        pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=10)

        #求取数据长度
        kk = len(self.qq_list)
        #进度条显示时间间隔
        show_time = int(kk*self.T*10)

        #运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg1 = "MainWindon run!;"
        self.textEdit.setText(msg1)
        t1.start()

        #设置LCD,用Qtimer开线程处理（线程2）
        self.lcdNumber.setDigitCount(5)  #设置显示数字个数
        self.lcdNumber.setMode(QLCDNumber.Dec) #设置十进制显示
        self.step_l = 0
        self.timer_l = QTimer()
        self.timer_l.timeout.connect(self.lcd_show)
        self.timer_l.start(1000)

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
            if(self.flag == 0):
                self.timer_l.stop()
                self.timer_p.stop()
                self.timer_plot.stop()
                break
            #发送数据
            command_data = Float64MultiArray()
            if (k < kk):
                command_data.data = self.qq_list[k, 0:self.n]
            else:
                command_data.data = self.qq_list[-1, 0:self.n]
            pub.publish(command_data)
            if(k%10==0):
                if(self.n == 7):
                    pub_msg = "armc" + "第" + str(k) + "次" + "publisher data is: " + '\n'\
                          "q1:" + str(command_data.data[0]) + '\n' \
                            "q2:" + str(command_data.data[1]) + '\n' \
                              "q3:" + str(command_data.data[2]) + '\n'\
                                "q4:" + str(command_data.data[3]) + '\n' \
                                   "q5:" + str(command_data.data[4]) + '\n' \
                                     "q6:" + str(command_data.data[5]) + '\n' \
                                        "q7:" + str(command_data.data[6]) + '\n'
                else:
                    pub_msg = "UR5" + "第" + str(k) + "次" + "publisher data is: " + '\n' \
                        "q1:" + str(command_data.data[0]) + '\n' \
                            "q2:" + str(command_data.data[1]) + '\n' \
                                "q3:" + str(command_data.data[2]) + '\n' \
                                    "q4:" + str(command_data.data[3]) + '\n' \
                                        "q5:" + str(command_data.data[4]) + '\n' \
                                            "q6:" + str(command_data.data[5]) + '\n'
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

#***********************************子窗口***************************************#
# ================阻抗控制窗口1================#
class ImpWindow1(QMainWindow, Ui_ImpForm1):
    # 建立全局变量
    state_f_list = list(np.zeros([1000, 6]))
    state_xx_list = list(np.zeros([1000, 6]))
    state_qq_list = list(np.zeros([1000, 7]))
    state_t_list = list(np.zeros(1000))
    state_t = 0
    #接受到的数据
    f_state = np.zeros(6)
    flag = 1  #
    arm_flag = False
    real_flag = False
    sub_pos_path = "/robot3/joint_states"
    sub_force_path = "/robot3/ft_sensor_topic"
    pub_path = "/robot3/armc_position_controller/command"
    T = 0.01
    n = 7  # 机械臂关节数

    def __init__(self, parent=None):
        super(ImpWindow1, self).__init__(parent)
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
        self.button_init.clicked.connect(self.read_init_point)
        self.checkBox_UR5.stateChanged.connect(self.armc_or_ur5)
        self.button_read.clicked.connect(self.read_paramter)

    # ===============按钮功能模块相关函数================#
    # 采用pyqtgraph绘制曲线,添加画板
    def set_graph_ui(self):
        pg.setConfigOptions(antialias=True)  # pg全局变量设置函数，antialias=True开启曲线抗锯齿

        win1 = pg.GraphicsLayoutWidget()  # 创建pg layout，可实现数据界面布局自动管理
        win2 = pg.GraphicsLayoutWidget()

        # pg绘图窗口可以作为一个widget添加到GUI中的graph_layout，当然也可以添加到Qt其他所有的容器中
        self.horizontalLayout_1.addWidget(win1)
        self.horizontalLayout_2.addWidget(win2)

        p1 = win1.addPlot(title="TCP")  # 添加第一个绘图窗口
        p1.setLabel('left', text='pos/mm', color='#ffffff')  # y轴设置函数
        p1.showGrid(x=True, y=True)  # 栅格设置函数
        p1.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p1.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p1.addLegend(size=(50, 30))  # 可选择是否添加legend

        p2 = win2.addPlot(title="F")  # 添加第一个绘图窗口
        p2.setLabel('left', text='F:N', color='#ffffff')  # y轴设置函数
        p2.showGrid(x=True, y=True)  # 栅格设置函数
        p2.setLogMode(x=False, y=False)  # False代表线性坐标轴，True代表对数坐标轴
        p2.setLabel('bottom', text='time', units='s')  # x轴设置函数
        p2.addLegend(size=(50, 30))
        return p1, p2

    # 绘画关节角和关节角速度曲线
    def plot_force(self, t, f, xx):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t, xx[:, 0], pen='b', name='x1', clear=True)
        self.p1.plot(t, xx[:, 1], pen='g', name='x2', clear=False)
        self.p1.plot(t, xx[:, 2], pen='r', name='x3', clear=False)
        self.p1.plot(t, xx[:, 3], pen='c', name='x4', clear=False)
        self.p1.plot(t, xx[:, 4], pen='m', name='x5', clear=False)
        self.p1.plot(t, xx[:, 5], pen='y', name='x6', clear=False)

        # 绘制操作空间位置图
        self.p2.plot(t, f[:, 0], pen='b', name='f1', clear=True)
        self.p2.plot(t, f[:, 1], pen='g', name='f2', clear=False)
        self.p2.plot(t, f[:, 2], pen='r', name='f3', clear=False)
        self.p2.plot(t, f[:, 3], pen='c', name='f4', clear=False)
        self.p2.plot(t, f[:, 4], pen='m', name='f5', clear=False)
        self.p2.plot(t, f[:, 5], pen='y', name='f6', clear=False)

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

        #读取话题地址
        self.sub_f_path = self.lineEdit_sub_f.text()
        self.sub_qq_path = self.lineEdit_sub_qq.text()
        self.pub_qq_path = self.lineEdit_pub_qq.text()

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

        # msg_path = "话题名称\n" + \
        #             "订阅末端六维力：" + self.sub_f_path + "\n" + \
        #             "订阅关节角度：" + self.sub_qq_path + "\n" + \
        #             "发送关节角度：" + self.pub_qq_path + "\n"

        msg = msg_joint + msg_pos + msg_force + msg_imp# + msg_path
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

        # 数据预处理
        qq_list = np.array(self.state_qq_list[-4:-1])
        qq_p = gf.joint_pos_Pretreatment(qq, qq_list, self.T)
        # 正运动计算
        xx = gf.get_begin_point(qq_p, self.flag)

        # 存储数据
        self.qq_state = np.copy(qq_p)
        self.state_t = self.state_t + self.T
        self.state_qq_list.append(self.qq_state)
        self.state_t_list.append(self.state_t)
        self.state_xx_list.append(xx)
        # 仅记录1000个数据点
        del self.state_t_list[0]
        del self.state_xx_list[0]
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

    def off(self):
        self.flag = 0

    def realtime_plot(self):
        plot_t = np.array(self.state_t_list)
        plot_f = np.array(self.state_f_list)
        plot_xx = np.array(self.state_xx_list)
        self.plot_force(plot_t, plot_f, plot_xx)

    def armc_or_ur5(self):
        # 运行该函数切换到ur5
        self.sub_pos_path = "/robot1/joint_states"
        self.sub_force_path = "/robot1/ft_sensor_topic"
        self.pub_path = "/robot1/ur5_position_controller/command"
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

        # 创建阻抗自适应阻抗实例
        imp_arm1 = imp.IMPController()

        #输入参数
        [DH0, q_max, q_min] = gf.get_robot_parameter(self.arm_flag)
        imp_arm1.get_robot_parameter(DH0, q_max, q_min,)
        imp_arm1.get_period(self.T)


        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg1 = "MainWindon run!;"
        self.textEdit.setText(msg1)
        t1.start()

        # 设置ProgressBar,用Qtimer开线程处理（线程3）
        self.step_p = 0
        self.timer_p = QTimer()
        self.timer_p.timeout.connect(self.probar_show)
        self.timer_p.start(100)

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
            #实时调整阻抗参数
            imp_arm1.get_imp_parameter(self.M, self.B, self.K, self.I)
            #读取期望力和关节角
            imp_arm1.get_expect_force(self.f_d)
            imp_arm1.get_expect_pos(self.xx_d)
            #读取当前关节角和力
            imp_arm1.get_current_joint(self.qq_state)
            imp_arm1.get_current_force(self.f_state)
            #计算修正关节角
            qr = imp_arm1.compute_imp_joint()
            # 发送数据
            command_data = Float64MultiArray()
            command_data.data = qr
            pub.publish(command_data)
            #显示数据
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

# ================阻抗控制窗口2================#
class ImpWindow2(QMainWindow, Ui_ImpForm2):
    def __init__(self, parent=None):
        super(ImpWindow2, self).__init__(parent)
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
    def __init__(self, parent=None):
        super(TeachWindow1, self).__init__(parent)
        self.setupUi(self)
        self.initUI()
        self.state_qq_list = list(np.zeros([1000, 7]))
        self.state_xx_list = list(np.zeros([1000, 3]))
        self.state_t_list = list(np.zeros(1000))
        self.state_t = 0
        self.T = 0.01
        self.flag = 1  # 开始或停止标签
        self.arm_flag = False
        self.gazebo_flag = 0
        self.sub_path = "/robot3/joint_states"
        self.pub_path = "/robot3/armc_position_controller/command"
        self.n = 7  # 机械臂关节数
        #示教器标签
        self.teach_flag = False
        self.teach_creat = False

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
        self.checkBox_UR5.stateChanged.connect(self.armc_or_ur5)
        self.button_teach.clicked.connect(self.create_teaching)
        self.button_begin_teach.clicked.connect(self.begin_teaching)
        self.button_end_teach.clicked.connect(self.end_teaching)
        self.button_teach_show.clicked.connect(self.show_msg)

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
    def create_teaching(self):
        #获取存储地址
        self.data_path = self.lineEdit_data_dir.text()
        # 建立示教器
        self.teaching = tl.teaching(self.data_path)
        self.teach_creat = True
        msg = "成功创建示教器\n"
        self.textEdit_1.setText(msg)

    def begin_teaching(self):
        self.teach_flag = True

    def end_teaching(self):
        self.teach_flag = False
        self.teaching.write_data()

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        #记录示教信息
        if(self.teach_creat == True and self.teach_flag == True):
            self.teaching.get_joint_position(qq)

        # 正运动计算
        xx = gf.get_begin_point(qq, self.flag)

        # 存储数据
        self.state_t = self.state_t + self.T
        self.state_qq = np.copy(qq)
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
        rospy.init_node('MainWindonRun')
        rospy.Subscriber(self.sub_path, JointState, self.joint_callback)
        pub = rospy.Publisher(self.pub_path, Float64MultiArray, queue_size=10)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg1 = "MainWindonRun"
        self.textEdit_1.setText(msg1)
        t1.start()

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
            command_data.data = np.zeros(self.n)
            pub.publish(command_data)
            QApplication.processEvents()
            k = k + 1
            rate.sleep()
    def show_msg(self):
        data = fo.read(self.data_path)
        self.textEdit_1.setText(str(data))

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

