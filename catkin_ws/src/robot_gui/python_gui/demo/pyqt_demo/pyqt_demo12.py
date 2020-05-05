# -*- coding: utf-8 -*-

"""
PyQt5 tutorial

In this example, we dispay an image
on the window.

author: py40.com
last edited: 2017年3月
"""
import sys
from PyQt5.QtWidgets import (QWidget, QHBoxLayout,
                             QLabel, QApplication)
from PyQt5.QtGui import QPixmap


class Example(QWidget):
    def __init__(self):
        super(QWidget,self).__init__()

        self.initUI()

    def initUI(self):
        hbox = QHBoxLayout(self)       #竖直建立标签
        pixmap = QPixmap("web.png")    #像素映射

        lbl = QLabel(self)             #建立标签
        lbl.setPixmap(pixmap)          #在标签中建立映射

        hbox.addWidget(lbl)            #竖直添加标签
        self.setLayout(hbox)

        self.move(300, 200)
        self.setWindowTitle('Red Rock')
        self.show()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())