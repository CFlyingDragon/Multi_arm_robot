# -*- coding: utf-8 -*-
#!/usr/bin/env python

import sys
#导入ros相关的模块
import rospy
from std_msgs.msg import String
#导入pyqt相关模块
from PyQt5.QtWidgets import (QWidget, QLabel, QVBoxLayout, QHBoxLayout,
                             QSlider, QPushButton, QApplication)
from PyQt5.QtCore import Qt

#建立界面
class PyGui(QWidget):
    def __init__(self):
        super(self, QWidget).__init__()

        #建立一个话题
        self.pub = rospy.Publisher("pyqt_topic", String, queue_size=10)
        rospy.init_node('pyqt_gui')
        self.current_value = 0                      #建立变量

        #建立外图标
        my_layout = QHBoxLayout()
        #建立按钮
        my_btn = QPushButton()
        my_btn.setText("Publisher")
        my_btn.setFixedWidth(130)
        my_btn.clicked.connect(self.publish_topic)
        my_layout.addWidget(my_btn)
        my_layout.addSpacing(50)
        #建立标签
        self.my_label = QLabel()
        self.my_label.setFixedWidth(140)
        self.my_label.setText("num: " + str(0))
        self.my_label.setEnabled(False)
        my_layout.addWidget(self.my_label)
        #建立滑条
        my_slider = QSlider()
        my_slider.setMinimum(0)
        my_slider.setMaximum(99)
        my_slider.setOrientation(Qt.Horizontal)
        my_slider.valueChanged.connect(self.changeValue)
        #水平布局
        my_vlay = QVBoxLayout()
        my_vlay.addWidget(my_slider)
        layout = QVBoxLayout()
        layout.addLayout(my_layout)
        layout.addLayout(my_vlay)
        self.setLayout(layout)
        #设置
        self.setGeometry(300, 300, 300, 200)
        self.setWindowTitle('Tooltips')
        self.show()

    def publish_topic(self):
        self.pub.publish(str(self.current_value))

    def changeValue(self, value):
        self.my_label.setText("num: " + str(value))
        self.current_value = value


if __name__ == "__main__":
    app = QApplication(sys.argv)
    sys.exit(app.exec_())