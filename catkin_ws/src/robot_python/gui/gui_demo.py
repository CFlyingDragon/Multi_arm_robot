#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于GUI开发界面函数：示例代码
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020.12.21

#系统函数
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
from robot_msg.srv import *

#绘图函数
import pyqtgraph as pg

#线程函数
import threading

#界面函数
from main_windon import Ui_MainWindow
from armc_form1 import Ui_ArmcForm1

#自定义文件
import gui_function_demo as gf

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
        armcMenu = menubar.addMenu('&自制Armc')

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

        # -------------------Armc菜单-------------------#
        #armc机械臂关节空间规划
        openArmc1 = QAction(QIcon('exit.png'), '关节控制', self)
        openArmc1.setShortcut('Ctrl+c')
        openArmc1.setStatusTip('自制机械臂关节空间规划和控制')
        openArmc1.triggered.connect(self.gotoArmc1)
        armcMenu.addAction(openArmc1)

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
    def gotoArmc1(self):
        self.hide()
        self.armc1 = ArmcWindow1()
        self.armc1.show()

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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MainWindow()
    myWin.show()
    sys.exit(app.exec_())