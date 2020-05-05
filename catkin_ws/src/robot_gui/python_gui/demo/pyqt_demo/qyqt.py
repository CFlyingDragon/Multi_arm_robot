#!/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets import QApplication, QWidget,QMainWindow  # 导入相应的包

class Example(QMainWindow):
    def __init__(self):
        self.initUI()

    def initUI(self):
        self.statusBar().showMessage('Ready')

        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Statusbar')
        self.show()

if __name__ == '__main__':
    # 创建QApplication对象是必须，管理整个>程序，参数可有可无，有的话可接收命令行参数
    app = QApplication(sys.argv)

    w = QWidget()  # 创建窗体对象，
    w.resize(250, 150)  # 设置窗体大小
    w.move(100, 300)  # 设置在屏幕上的显示位置
    w.setWindowTitle('Simple')  # 设置窗口标题
    w.show()  # 窗口显示
    #ex = Example()

    sys.exit(app.exec_())
