# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'plot_armctr_form.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_PlotArmctrForm(object):
    def setupUi(self, PlotArmctrForm):
        PlotArmctrForm.setObjectName("PlotArmctrForm")
        PlotArmctrForm.resize(1238, 903)
        PlotArmctrForm.setIconSize(QtCore.QSize(16, 16))
        PlotArmctrForm.setAnimated(False)
        self.centralwidget = QtWidgets.QWidget(PlotArmctrForm)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 451, 261))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout_1 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout_1.setObjectName("horizontalLayout_1")
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(10, 270, 451, 301))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.horizontalLayoutWidget_5 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_5.setGeometry(QtCore.QRect(460, 410, 511, 431))
        self.horizontalLayoutWidget_5.setObjectName("horizontalLayoutWidget_5")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_5)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.frame = QtWidgets.QFrame(self.horizontalLayoutWidget_5)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.textEdit = QtWidgets.QTextEdit(self.frame)
        self.textEdit.setGeometry(QtCore.QRect(10, 30, 461, 211))
        self.textEdit.setObjectName("textEdit")
        self.label = QtWidgets.QLabel(self.frame)
        self.label.setGeometry(QtCore.QRect(160, 0, 131, 31))
        self.label.setObjectName("label")
        self.button_receive = QtWidgets.QPushButton(self.frame)
        self.button_receive.setGeometry(QtCore.QRect(130, 260, 221, 61))
        self.button_receive.setIconSize(QtCore.QSize(16, 16))
        self.button_receive.setAutoRepeat(False)
        self.button_receive.setAutoExclusive(False)
        self.button_receive.setAutoRepeatDelay(400)
        self.button_receive.setAutoRepeatInterval(200)
        self.button_receive.setAutoDefault(True)
        self.button_receive.setFlat(False)
        self.button_receive.setObjectName("button_receive")
        self.button_storage_begin = QtWidgets.QPushButton(self.frame)
        self.button_storage_begin.setGeometry(QtCore.QRect(60, 340, 101, 51))
        self.button_storage_begin.setIconSize(QtCore.QSize(16, 16))
        self.button_storage_begin.setAutoRepeat(False)
        self.button_storage_begin.setAutoExclusive(False)
        self.button_storage_begin.setAutoRepeatDelay(400)
        self.button_storage_begin.setAutoRepeatInterval(200)
        self.button_storage_begin.setAutoDefault(True)
        self.button_storage_begin.setFlat(False)
        self.button_storage_begin.setObjectName("button_storage_begin")
        self.button_storage_end = QtWidgets.QPushButton(self.frame)
        self.button_storage_end.setGeometry(QtCore.QRect(300, 340, 101, 51))
        self.button_storage_end.setIconSize(QtCore.QSize(16, 16))
        self.button_storage_end.setAutoRepeat(False)
        self.button_storage_end.setAutoExclusive(False)
        self.button_storage_end.setAutoRepeatDelay(400)
        self.button_storage_end.setAutoRepeatInterval(200)
        self.button_storage_end.setAutoDefault(True)
        self.button_storage_end.setFlat(False)
        self.button_storage_end.setObjectName("button_storage_end")
        self.horizontalLayout_5.addWidget(self.frame)
        self.horizontalLayoutWidget_6 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_6.setGeometry(QtCore.QRect(460, 10, 771, 401))
        self.horizontalLayoutWidget_6.setObjectName("horizontalLayoutWidget_6")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_6)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.frame_4 = QtWidgets.QFrame(self.horizontalLayoutWidget_6)
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.lineEdit_force_armt = QtWidgets.QLineEdit(self.frame_4)
        self.lineEdit_force_armt.setGeometry(QtCore.QRect(120, 250, 611, 41))
        self.lineEdit_force_armt.setObjectName("lineEdit_force_armt")
        self.label_30 = QtWidgets.QLabel(self.frame_4)
        self.label_30.setGeometry(QtCore.QRect(10, 250, 91, 33))
        self.label_30.setObjectName("label_30")
        self.lineEdit_force_armrc = QtWidgets.QLineEdit(self.frame_4)
        self.lineEdit_force_armrc.setGeometry(QtCore.QRect(120, 310, 611, 41))
        self.lineEdit_force_armrc.setObjectName("lineEdit_force_armrc")
        self.label_31 = QtWidgets.QLabel(self.frame_4)
        self.label_31.setGeometry(QtCore.QRect(10, 310, 91, 33))
        self.label_31.setObjectName("label_31")
        self.label_11 = QtWidgets.QLabel(self.frame_4)
        self.label_11.setGeometry(QtCore.QRect(330, 0, 101, 31))
        self.label_11.setObjectName("label_11")
        self.lineEdit_pos_armt = QtWidgets.QLineEdit(self.frame_4)
        self.lineEdit_pos_armt.setGeometry(QtCore.QRect(120, 80, 611, 41))
        self.lineEdit_pos_armt.setObjectName("lineEdit_pos_armt")
        self.label_32 = QtWidgets.QLabel(self.frame_4)
        self.label_32.setGeometry(QtCore.QRect(10, 130, 111, 33))
        self.label_32.setObjectName("label_32")
        self.lineEdit_pos_armc = QtWidgets.QLineEdit(self.frame_4)
        self.lineEdit_pos_armc.setGeometry(QtCore.QRect(120, 130, 611, 41))
        self.lineEdit_pos_armc.setObjectName("lineEdit_pos_armc")
        self.label_33 = QtWidgets.QLabel(self.frame_4)
        self.label_33.setGeometry(QtCore.QRect(10, 80, 111, 33))
        self.label_33.setObjectName("label_33")
        self.label_34 = QtWidgets.QLabel(self.frame_4)
        self.label_34.setGeometry(QtCore.QRect(10, 30, 111, 33))
        self.label_34.setObjectName("label_34")
        self.lineEdit_pos_armr = QtWidgets.QLineEdit(self.frame_4)
        self.lineEdit_pos_armr.setGeometry(QtCore.QRect(120, 30, 611, 41))
        self.lineEdit_pos_armr.setObjectName("lineEdit_pos_armr")
        self.label_35 = QtWidgets.QLabel(self.frame_4)
        self.label_35.setGeometry(QtCore.QRect(10, 190, 91, 33))
        self.label_35.setObjectName("label_35")
        self.lineEdit_force_armr = QtWidgets.QLineEdit(self.frame_4)
        self.lineEdit_force_armr.setGeometry(QtCore.QRect(120, 190, 611, 41))
        self.lineEdit_force_armr.setObjectName("lineEdit_force_armr")
        self.horizontalLayout_6.addWidget(self.frame_4)
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(970, 410, 261, 431))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame_6 = QtWidgets.QFrame(self.verticalLayoutWidget)
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.label_3 = QtWidgets.QLabel(self.frame_6)
        self.label_3.setGeometry(QtCore.QRect(100, 0, 81, 31))
        self.label_3.setObjectName("label_3")
        self.label_9 = QtWidgets.QLabel(self.frame_6)
        self.label_9.setGeometry(QtCore.QRect(10, 40, 91, 33))
        self.label_9.setObjectName("label_9")
        self.lineEdit_T = QtWidgets.QLineEdit(self.frame_6)
        self.lineEdit_T.setGeometry(QtCore.QRect(100, 40, 121, 33))
        self.lineEdit_T.setObjectName("lineEdit_T")
        self.button_begin = QtWidgets.QPushButton(self.frame_6)
        self.button_begin.setGeometry(QtCore.QRect(10, 100, 221, 61))
        self.button_begin.setIconSize(QtCore.QSize(16, 16))
        self.button_begin.setAutoRepeat(False)
        self.button_begin.setAutoExclusive(False)
        self.button_begin.setAutoRepeatDelay(400)
        self.button_begin.setAutoRepeatInterval(200)
        self.button_begin.setAutoDefault(True)
        self.button_begin.setFlat(False)
        self.button_begin.setObjectName("button_begin")
        self.button_stop = QtWidgets.QPushButton(self.frame_6)
        self.button_stop.setGeometry(QtCore.QRect(10, 180, 221, 61))
        self.button_stop.setAutoDefault(True)
        self.button_stop.setObjectName("button_stop")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.frame_6)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(30, 280, 191, 80))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.checkBox_real = QtWidgets.QCheckBox(self.verticalLayoutWidget_2)
        self.checkBox_real.setObjectName("checkBox_real")
        self.verticalLayout_2.addWidget(self.checkBox_real)
        self.checkBox_force = QtWidgets.QCheckBox(self.verticalLayoutWidget_2)
        self.checkBox_force.setObjectName("checkBox_force")
        self.verticalLayout_2.addWidget(self.checkBox_force)
        self.verticalLayout.addWidget(self.frame_6)
        self.horizontalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_3.setGeometry(QtCore.QRect(10, 570, 451, 271))
        self.horizontalLayoutWidget_3.setObjectName("horizontalLayoutWidget_3")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        PlotArmctrForm.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(PlotArmctrForm)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1238, 31))
        self.menubar.setDefaultUp(False)
        self.menubar.setObjectName("menubar")
        PlotArmctrForm.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(PlotArmctrForm)
        self.statusbar.setObjectName("statusbar")
        PlotArmctrForm.setStatusBar(self.statusbar)

        self.retranslateUi(PlotArmctrForm)
        QtCore.QMetaObject.connectSlotsByName(PlotArmctrForm)

    def retranslateUi(self, PlotArmctrForm):
        _translate = QtCore.QCoreApplication.translate
        PlotArmctrForm.setWindowTitle(_translate("PlotArmctrForm", "MainWindow"))
        self.label.setText(_translate("PlotArmctrForm", "运行内容显示框"))
        self.button_receive.setText(_translate("PlotArmctrForm", "接收机器人状态"))
        self.button_storage_begin.setText(_translate("PlotArmctrForm", "开始存储"))
        self.button_storage_end.setText(_translate("PlotArmctrForm", "结束存储"))
        self.lineEdit_force_armt.setText(_translate("PlotArmctrForm", "/home/d/catkin_ws/src/robot_bag/armctr/armt_force.txt"))
        self.label_30.setText(_translate("PlotArmctrForm", "Armt力路径:"))
        self.lineEdit_force_armrc.setText(_translate("PlotArmctrForm", "/home/d/catkin_ws/src/robot_bag/armctr/armc_force.txt"))
        self.label_31.setText(_translate("PlotArmctrForm", "Armc路径:"))
        self.label_11.setText(_translate("PlotArmctrForm", "数据存储地址"))
        self.lineEdit_pos_armt.setText(_translate("PlotArmctrForm", "/home/d/catkin_ws/src/robot_bag/armctr/armt_position.txt"))
        self.label_32.setText(_translate("PlotArmctrForm", "Armc位置路径:"))
        self.lineEdit_pos_armc.setText(_translate("PlotArmctrForm", "/home/d/catkin_ws/src/robot_bag/armctr/armc_position.txt"))
        self.label_33.setText(_translate("PlotArmctrForm", "Armt位置路径："))
        self.label_34.setText(_translate("PlotArmctrForm", "UR5位置路径："))
        self.lineEdit_pos_armr.setText(_translate("PlotArmctrForm", "/home/d/catkin_ws/src/robot_bag/armctr/armr_position.txt"))
        self.label_35.setText(_translate("PlotArmctrForm", "Ur5力路径:"))
        self.lineEdit_force_armr.setText(_translate("PlotArmctrForm", "/home/d/catkin_ws/src/robot_bag/armctr/armr_force.txt"))
        self.label_3.setText(_translate("PlotArmctrForm", "绘制周期"))
        self.label_9.setText(_translate("PlotArmctrForm", "接收周期："))
        self.lineEdit_T.setText(_translate("PlotArmctrForm", "0.01"))
        self.button_begin.setText(_translate("PlotArmctrForm", "开始"))
        self.button_stop.setText(_translate("PlotArmctrForm", "停止"))
        self.checkBox_real.setText(_translate("PlotArmctrForm", "实物勾      仿真默认"))
        self.checkBox_force.setText(_translate("PlotArmctrForm", "六维力勾      位置默认"))

