# -*- coding: utf-8 -*-

"""
PyQt5 tutorial

This example draws three rectangles in three
#different colours.

author: py40.com
last edited: 2017年3月
"""
import sys
from PyQt5.QtWidgets import QWidget, QApplication
from PyQt5.QtGui import QPainter, QColor, QBrush


class Example(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.initUI()

    def initUI(self):
        self.setGeometry(300, 300, 350, 100)
        self.setWindowTitle('Colours')
        self.show()

    def paintEvent(self, e):
        qp = QPainter()
        qp.begin(self)
        self.drawRectangles(qp)
        qp.end()

    #绘制矩阵
    def drawRectangles(self, qp):
        col = QColor(0, 0, 0)                  #设置颜色
        col.setNamedColor('#d4d4d4')           #设置颜色名字
        qp.setPen(col)                         #设置笔的颜色

        qp.setBrush(QColor(200, 0, 0))         #设置填充颜色
        qp.drawRect(10, 15, 90, 60)            #绘制矩形

        qp.setBrush(QColor(255, 80, 0, 160))
        qp.drawRect(130, 15, 90, 60)

        qp.setBrush(QColor(25, 0, 90, 200))
        qp.drawRect(250, 15, 90, 60)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())