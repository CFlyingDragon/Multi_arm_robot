# -*- coding: utf-8 -*-

"""
PyQt5 tutorial

In this example, we determine the event sender
object.

author: py40.com
last edited: 2017年3月
"""

import sys
from PyQt5.QtWidgets import (QWidget, QPushButton, QLineEdit,
                             QInputDialog, QApplication)


class Example(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.initUI()

    def initUI(self):
        self.btn = QPushButton('Dialog', self)     #建立按钮
        self.btn.move(20, 20)
        self.btn.clicked.connect(self.showDialog)  #点击触发，连接到函数

        self.le = QLineEdit(self)                  #编辑线框
        self.le.move(130, 22)

        self.setGeometry(300, 300, 290, 150)
        self.setWindowTitle('Input dialog')
        self.show()

    def showDialog(self):
        #获取文本
        text, ok = QInputDialog.getText(self, 'Input Dialog',
                                        'Enter your name:')

        if ok:
            #设置文本到编辑线框 
            self.le.setText(str(text))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())