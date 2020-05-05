# -*- coding: utf-8 -*-

"""
PyQt5 tutorial

In this example, we determine the event sender
object.

author: py40.com
last edited: 2017年3月
"""
import sys
from PyQt5.QtWidgets import QWidget, QCheckBox, QApplication
from PyQt5.QtCore import Qt


class Example(QWidget):
    def __init__(self):
        super(QWidget,self).__init__()

        self.initUI()

    def initUI(self):
        #建立复选框
        cb = QCheckBox('Show title', self)
        cb.move(20, 20)
        #设置构造函数
        cb.toggle()
        #信号源为复选框变化，连接到标题函数
        cb.stateChanged.connect(self.changeTitle)

        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('QCheckBox')
        self.show()

    def changeTitle(self, state):
        #状态，判断是否勾选
        if state == Qt.Checked:
            self.setWindowTitle('QCheckBox')
        else:
            self.setWindowTitle('')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())