#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于实时显示和存储参数
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2020年6月13号
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
import time

#界面函数
from plot_main_windon import Ui_PlotMainWindow

#自定义文件
import gui_function as gf
from robot_python import FileOpen as fo

#**********************************主窗口***************************************#
class PlotWindow(QMainWindow, Ui_PlotMainWindow):
    #建立全局变量
    state_qq_list = list(np.zeros([100,7]))
    state_f_list = list(np.zeros([100,6]))
    state_t_list = list(np.zeros(100))
    state_t = 0

    #构造函数
    def __init__(self, parent=None):
        super(PlotWindow, self).__init__(parent)
        self.T = 0.01
        #关节绘制选择
        self.q1_flag = False
        self.q2_flag = False
        self.q3_flag = False
        self.q4_flag = False
        self.q5_flag = False
        self.q6_flag = False
        self.q7_flag = False
        self.qq_flag = False

        #力绘制选择
        self.f1_flag = False
        self.f2_flag = False
        self.f3_flag = False
        self.f4_flag = False
        self.f5_flag = False
        self.f6_flag = False
        self.ff_flag = False

        #存储标签
        self.storage_flag = False

        self.sub_force_path = "/ft_sensor_topic"
        self.sub_pos_path = "/armc/joint_states"
        self.n = 7  # 机械臂关节数

        self.setupUi(self)
        self.initUI()

    #初始化UI
    def initUI(self):
        # ======================菜单栏功能模块=======================#
        #创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')

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

        # =======================绘图相关设置=======================#
        self.p1, self.p2 = self.set_graph_ui()  # 设置绘图窗口

        # =======================按钮功能模块=======================#
        self.button_begin.clicked.connect(self.begin_function)
        self.button_stop.clicked.connect(self.stop)
        self.button_refresh.clicked.connect(self.refresh_radioButton)
        self.button_receive.clicked.connect(self.run_topic)
        self.button_storage_begin.clicked.connect(self.storage_begin)
        self.button_storage_end.clicked.connect(self.storage_end)

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

    #绘画所有关节曲线
    def plot_joint_all(self, t, qq):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t, qq[:, 0], pen='b', name='qq1', clear=True)
        self.p1.plot(t, qq[:, 1], pen='g', name='qq2', clear=False)
        self.p1.plot(t, qq[:, 2], pen='r', name='qq3', clear=False)
        self.p1.plot(t, qq[:, 3], pen='c', name='qq4', clear=False)
        self.p1.plot(t, qq[:, 4], pen='m', name='qq5', clear=False)
        self.p1.plot(t, qq[:, 5], pen='y', name='qq6', clear=False)
        self.p1.plot(t, qq[:, 6], pen='w', name='qq7', clear=False)

    # 绘画所有力曲线
    def plot_force_all(self,t, f):
        # 绘制六维力图
        self.p2.plot(t, f[:, 0], pen='b', name='F1', clear=True)
        self.p2.plot(t, f[:, 1], pen='g', name='F2', clear=False)
        self.p2.plot(t, f[:, 2], pen='r', name='F3', clear=False)
        self.p2.plot(t, f[:, 3], pen='c', name='F4', clear=False)
        self.p2.plot(t, f[:, 4], pen='m', name='F5', clear=False)
        self.p2.plot(t, f[:, 5], pen='y', name='F6', clear=False)

    # 绘画单个关节曲线
    def plot_joint(self, t, qq, name):
        # 绘制位置图,表示颜色的单字符串（b，g，r，c，m，y，k，w）
        self.p1.plot(t, qq, pen='b', name=name, clear=True)

    # 绘画单个力曲线
    def plot_force(self, t, f, name):
        # 绘制六维力图
        self.p2.plot(t, f, pen='b', name=name, clear=True)

    #刷新机选择按钮
    def refresh_radioButton(self):
        self.q1_flag = self.radioButton_q1.isChecked()
        self.q2_flag = self.radioButton_q2.isChecked()
        self.q3_flag = self.radioButton_q3.isChecked()
        self.q4_flag = self.radioButton_q4.isChecked()
        self.q5_flag = self.radioButton_q5.isChecked()
        self.q6_flag = self.radioButton_q6.isChecked()
        self.q7_flag = self.radioButton_q7.isChecked()
        self.qq_flag = self.radioButton_qq.isChecked()

        self.f1_flag = self.radioButton_f1.isChecked()
        self.f2_flag = self.radioButton_f2.isChecked()
        self.f3_flag = self.radioButton_f3.isChecked()
        self.f4_flag = self.radioButton_f4.isChecked()
        self.f5_flag = self.radioButton_f5.isChecked()
        self.f6_flag = self.radioButton_f6.isChecked()
        self.ff_flag = self.radioButton_f.isChecked()

        msg = "按钮状态已刷新！\n"
        self.textEdit.setText(msg)

    # 开始存储取数据
    def storage_begin(self):
        #获取写入地址
        self.storage_qq_path = str(self.lineEdit_data_qq.text())
        self.storage_f_path = str(self.lineEdit_data_f.text())

        # 存储变量清空
        self.storage_qq = []
        self.storage_f = []

        self.storage_flag = True
        msg = "开始写入数据！\n"
        self.textEdit.setText(msg)

    # 开始存储取数据
    def storage_end(self):
        self.storage_flag = False
        # 转换数据类型
        qq_data = np.array(self.storage_qq)
        force_data = np.array(self.storage_f)

        # 写入数据
        fo.write(qq_data, self.storage_qq_path)
        fo.write(force_data, self.storage_f_path)

        msg = "数据已写入：/home/d/catkin_ws/src/robot_bag/imp_data\n"
        self.textEdit.setText(msg)

    ##关节角订阅回调函数
    def joint_callback(self, msg):
        qq = np.zeros(self.n)
        for i in range(self.n):
            qq[i] = msg.position[i]

        # 存储数据
        if (self.storage_flag):
            self.storage_qq.append(qq)

        # 绘图数据
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

        # 绘图数据
        self.state_f_list.append(f)
        # 仅记录1000个数据点
        del self.state_f_list[0]

    ##末端力订阅线程
    def thread_spin(self):
        rospy.spin()

    def stop(self):
        self.run_flag = False

    def realtime_plot(self):
        # 将列表转换为数组
        plot_t = np.array(self.state_t_list)
        plot_qq = np.array(self.state_qq_list)
        plot_f = np.array(self.state_f_list)

        # 绘制关节图
        if (self.q1_flag):
            name = "q1"
            self.plot_joint(plot_t, plot_qq[:, 0], name)
        elif (self.q2_flag):
            name = "q2"
            self.plot_joint(plot_t, plot_qq[:, 1], name)
        elif (self.q3_flag):
            name = "q2"
            self.plot_joint(plot_t, plot_qq[:, 2], name)
        elif (self.q4_flag):
            name = "q4"
            self.plot_joint(plot_t, plot_qq[:, 3], name)
        elif (self.q5_flag):
            name = "q5"
            self.plot_joint(plot_t, plot_qq[:, 4], name)
        elif (self.q6_flag):
            name = "q6"
            self.plot_joint(plot_t, plot_qq[:, 5], name)
        elif (self.q7_flag):
            name = "q7"
            self.plot_joint(plot_t, plot_qq[:, 6], name)
        else:
            self.plot_joint_all(plot_t, plot_qq)

        # 绘制六维力图
        if (self.f1_flag):
            name = "f1"
            self.plot_force(plot_t, plot_f[:, 0], name)
        elif (self.f2_flag):
            name = "f2"
            self.plot_force(plot_t, plot_f[:, 1], name)
        elif (self.f3_flag):
            name = "f3"
            self.plot_force(plot_t, plot_f[:, 2], name)
        elif (self.f4_flag):
            name = "f4"
            self.plot_force(plot_t, plot_f[:, 3], name)
        elif (self.f5_flag):
            name = "f5"
            self.plot_force(plot_t, plot_f[:, 4], name)
        elif (self.f6_flag):
            name = "f6"
            self.plot_force(plot_t, plot_f[:, 5], name)
        else:
            self.plot_force_all(plot_t, plot_f)

    def run_topic(self):
        # 运行话题
        rospy.init_node('upper_controller_node')
        rospy.Subscriber(self.sub_pos_path, JointState, self.joint_callback)
        rospy.Subscriber(self.sub_force_path, WrenchStamped, self.force_callback)

        # 运行线程1,收话题线程
        t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
        msg_tip = "upper_controller_node run!"
        self.textEdit.setText(msg_tip)
        t1.start()

    def begin_function(self):
        #运行标签启动
        self.run_flag = True

        # 设置绘图,用Qtimer开线程处理（线程4）
        self.timer_plot = QTimer()
        self.timer_plot.timeout.connect(self.realtime_plot)
        self.timer_plot.start(1000)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            #判断是否停止
            if (not self.run_flag):
                self.timer_p.stop()
                self.timer_plot.stop()
                break

            rate.sleep()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = PlotWindow()
    myWin.show()
    sys.exit(app.exec_())

