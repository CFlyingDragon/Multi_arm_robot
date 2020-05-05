# -*- coding: utf-8 -*-

"""
PyQt5 tutorial

This example shows a QProgressBar widget.


author: py40.com
last edited: 2017年3月
"""
import sys
from PyQt5.QtWidgets import (QWidget, QProgressBar,
                             QPushButton, QApplication)
from PyQt5.QtCore import QBasicTimer


class Example(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.initUI()

    def initUI(self):
        #进度条
        self.pbar = QProgressBar(self)
        self.pbar.setGeometry(30, 40, 200, 25)

        #设置按钮
        self.btn = QPushButton('Start', self)
        self.btn.move(40, 80)
        self.btn.clicked.connect(self.doAction)

        #设置计时器
        self.timer = QBasicTimer()
        self.step = 0

        self.setGeometry(300, 300, 280, 170)
        self.setWindowTitle('QProgressBar')
        self.show()

        #时间计数器
    def timerEvent(self, e):

        if self.step >= 100:
            self.timer.stop()                       #计时器暂停
            self.btn.setText('Finished')            #设置按钮文本
            return

        self.step = self.step + 1                   #计数器加1
        self.pbar.setValue(self.step)               #设置进程值

        #执行动作
    def doAction(self):

        if self.timer.isActive():
            self.timer.stop()
            self.btn.setText('Start')
        else:
            self.timer.start(100, self)
            self.btn.setText('Stop')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())