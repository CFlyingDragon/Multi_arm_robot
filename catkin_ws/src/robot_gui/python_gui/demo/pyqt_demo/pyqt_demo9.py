# -*- coding: utf-8 -*-

"""
PyQt5 tutorial

In this example, we create three toggle buttons.
They will control the background color of a
QFrame.

author: py40.com
last edited: 2017年3月
"""
import sys
from PyQt5.QtWidgets import (QWidget, QPushButton,
                             QFrame, QApplication)
from PyQt5.QtGui import QColor


class Example(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.initUI()

    def initUI(self):

        #建立颜色，红绿蓝
        self.col = QColor(0, 0, 0)

        #建立按钮
        redb = QPushButton('Red', self)
        redb.setCheckable(True)                    #检测是否按下，默认设置为真
        redb.move(10, 10)

        redb.clicked[bool].connect(self.setColor)  #按下为否，建立信号槽

        greenb = QPushButton('Green', self)
        greenb.setCheckable(True)
        greenb.move(10, 60)

        greenb.clicked[bool].connect(self.setColor)

        blueb = QPushButton('Blue', self)
        blueb.setCheckable(True)
        blueb.move(10, 110)

        blueb.clicked[bool].connect(self.setColor)

        self.square = QFrame(self)
        self.square.setGeometry(150, 20, 100, 100)
        self.square.setStyleSheet("QWidget { background-color: %s }" %
                                  self.col.name())

        self.setGeometry(300, 300, 280, 170)
        self.setWindowTitle('Toggle button')
        self.show()

        #设置颜色
    def setColor(self, pressed):
        #获取信号源
        source = self.sender()

        #按下，为真
        if pressed:
            val = 255
        else:
            val = 0

        #设置红色
        if source.text() == "Red":
            self.col.setRed(val)
        #设置绿色
        elif source.text() == "Green":
            self.col.setGreen(val)
        #设置蓝色
        else:
            self.col.setBlue(val)

        self.square.setStyleSheet("QFrame { background-color: %s }" %
                                  self.col.name())


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())