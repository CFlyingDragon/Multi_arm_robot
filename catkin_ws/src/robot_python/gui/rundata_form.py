# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'rundata_form.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_RunForm(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1238, 903)
        MainWindow.setIconSize(QtCore.QSize(16, 16))
        MainWindow.setAnimated(False)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(10, 0, 451, 411))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout_1 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout_1.setObjectName("horizontalLayout_1")
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(10, 410, 451, 431))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.horizontalLayoutWidget_5 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_5.setGeometry(QtCore.QRect(460, 410, 501, 431))
        self.horizontalLayoutWidget_5.setObjectName("horizontalLayoutWidget_5")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_5)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.frame = QtWidgets.QFrame(self.horizontalLayoutWidget_5)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.textEdit = QtWidgets.QTextEdit(self.frame)
        self.textEdit.setGeometry(QtCore.QRect(10, 30, 461, 331))
        self.textEdit.setObjectName("textEdit")
        self.label = QtWidgets.QLabel(self.frame)
        self.label.setGeometry(QtCore.QRect(160, 0, 131, 31))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.frame)
        self.label_2.setGeometry(QtCore.QRect(10, 370, 41, 31))
        self.label_2.setObjectName("label_2")
        self.progressBar = QtWidgets.QProgressBar(self.frame)
        self.progressBar.setGeometry(QtCore.QRect(60, 370, 411, 31))
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.horizontalLayout_5.addWidget(self.frame)
        self.horizontalLayoutWidget_6 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_6.setGeometry(QtCore.QRect(460, 0, 501, 411))
        self.horizontalLayoutWidget_6.setObjectName("horizontalLayoutWidget_6")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_6)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.frame_4 = QtWidgets.QFrame(self.horizontalLayoutWidget_6)
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.label_11 = QtWidgets.QLabel(self.frame_4)
        self.label_11.setGeometry(QtCore.QRect(200, 0, 101, 31))
        self.label_11.setTextFormat(QtCore.Qt.AutoText)
        self.label_11.setObjectName("label_11")
        self.label_10 = QtWidgets.QLabel(self.frame_4)
        self.label_10.setGeometry(QtCore.QRect(10, 40, 171, 33))
        self.label_10.setObjectName("label_10")
        self.lineEdit_data_path = QtWidgets.QLineEdit(self.frame_4)
        self.lineEdit_data_path.setGeometry(QtCore.QRect(10, 80, 461, 33))
        self.lineEdit_data_path.setObjectName("lineEdit_data_path")
        self.label_14 = QtWidgets.QLabel(self.frame_4)
        self.label_14.setGeometry(QtCore.QRect(10, 120, 271, 33))
        self.label_14.setObjectName("label_14")
        self.lineEdit_pub_path = QtWidgets.QLineEdit(self.frame_4)
        self.lineEdit_pub_path.setGeometry(QtCore.QRect(10, 160, 461, 33))
        self.lineEdit_pub_path.setObjectName("lineEdit_pub_path")
        self.label_15 = QtWidgets.QLabel(self.frame_4)
        self.label_15.setGeometry(QtCore.QRect(10, 200, 341, 33))
        self.label_15.setObjectName("label_15")
        self.lineEdit_sub_pos = QtWidgets.QLineEdit(self.frame_4)
        self.lineEdit_sub_pos.setGeometry(QtCore.QRect(10, 240, 461, 33))
        self.lineEdit_sub_pos.setObjectName("lineEdit_sub_pos")
        self.label_16 = QtWidgets.QLabel(self.frame_4)
        self.label_16.setGeometry(QtCore.QRect(10, 290, 281, 33))
        self.label_16.setObjectName("label_16")
        self.lineEdit_sub_force = QtWidgets.QLineEdit(self.frame_4)
        self.lineEdit_sub_force.setGeometry(QtCore.QRect(10, 330, 461, 33))
        self.lineEdit_sub_force.setObjectName("lineEdit_sub_force")
        self.horizontalLayout_6.addWidget(self.frame_4)
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(960, 410, 271, 431))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame_6 = QtWidgets.QFrame(self.verticalLayoutWidget)
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.label_3 = QtWidgets.QLabel(self.frame_6)
        self.label_3.setGeometry(QtCore.QRect(60, 0, 161, 31))
        self.label_3.setObjectName("label_3")
        self.verticalLayout.addWidget(self.frame_6)
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(960, 0, 271, 411))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame_5 = QtWidgets.QFrame(self.verticalLayoutWidget_2)
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.button_begin = QtWidgets.QPushButton(self.frame_5)
        self.button_begin.setGeometry(QtCore.QRect(10, 20, 221, 61))
        self.button_begin.setIconSize(QtCore.QSize(16, 16))
        self.button_begin.setAutoRepeat(False)
        self.button_begin.setAutoExclusive(False)
        self.button_begin.setAutoRepeatDelay(400)
        self.button_begin.setAutoRepeatInterval(200)
        self.button_begin.setAutoDefault(True)
        self.button_begin.setFlat(False)
        self.button_begin.setObjectName("Button_begin")
        self.button_end = QtWidgets.QPushButton(self.frame_5)
        self.button_end.setGeometry(QtCore.QRect(10, 110, 221, 61))
        self.button_end.setAutoDefault(True)
        self.button_end.setObjectName("button_end")
        self.layoutWidget_2 = QtWidgets.QWidget(self.frame_5)
        self.layoutWidget_2.setGeometry(QtCore.QRect(20, 200, 201, 91))
        self.layoutWidget_2.setObjectName("layoutWidget_2")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.layoutWidget_2)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.checkBox_UR5 = QtWidgets.QCheckBox(self.layoutWidget_2)
        self.checkBox_UR5.setObjectName("checkBox_UR5")
        self.verticalLayout_4.addWidget(self.checkBox_UR5)
        self.checkBox_gazebo = QtWidgets.QCheckBox(self.layoutWidget_2)
        self.checkBox_gazebo.setObjectName("checkBox_gazebo")
        self.verticalLayout_4.addWidget(self.checkBox_gazebo)
        self.button_get_dir = QtWidgets.QPushButton(self.frame_5)
        self.button_get_dir.setGeometry(QtCore.QRect(10, 310, 221, 61))
        self.button_get_dir.setAutoDefault(True)
        self.button_get_dir.setObjectName("button_get_dir")
        self.verticalLayout_2.addWidget(self.frame_5)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1238, 35))
        self.menubar.setDefaultUp(False)
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "运行给定数据窗口"))
        self.label.setText(_translate("MainWindow", "运行内容显示框"))
        self.label_2.setText(_translate("MainWindow", "进度："))
        self.label_11.setText(_translate("MainWindow", "地址获取框"))
        self.label_10.setText(_translate("MainWindow", "数据地址data_path："))
        self.lineEdit_data_path.setText(_translate("MainWindow", "/home/d/catkin_ws/src/robot_python/data"))
        self.label_14.setText(_translate("MainWindow", "命令地址：pub_command_path："))
        self.lineEdit_pub_path.setText(_translate("MainWindow", "/robot3/armc_position_controller/command"))
        self.label_15.setText(_translate("MainWindow", "订阅关节角度地址：sub_pos_path："))
        self.lineEdit_sub_pos.setText(_translate("MainWindow", "/robot3/joint_states"))
        self.label_16.setText(_translate("MainWindow", "订阅六维力地址：sub_force_path："))
        self.lineEdit_sub_force.setText(_translate("MainWindow", "/robot3/ft_sensor_topic"))
        self.label_3.setText(_translate("MainWindow", "获取数据/话题地址"))
        self.button_begin.setText(_translate("MainWindow", "开始"))
        self.button_end.setText(_translate("MainWindow", "停止"))
        self.checkBox_UR5.setText(_translate("MainWindow", "UR5勾   armc默认"))
        self.checkBox_gazebo.setText(_translate("MainWindow", "实物勾   仿真默认"))
        self.button_get_dir.setText(_translate("MainWindow", "获取地址"))

