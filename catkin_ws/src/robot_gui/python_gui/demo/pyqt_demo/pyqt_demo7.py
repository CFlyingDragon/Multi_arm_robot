# -*- coding: utf-8 -*-

"""
PyQt5 tutorial

In this example, we determine the event sender
object.

author: py40.com
last edited: 2017年3月
"""
import sys
from PyQt5.QtWidgets import (QMainWindow, QTextEdit,
                             QAction, QFileDialog, QApplication)
from PyQt5.QtGui import QIcon


class Example(QMainWindow):
    def __init__(self):
        super(QMainWindow,self).__init__()

        self.initUI()

    def initUI(self):
        self.textEdit = QTextEdit()                         #文本编辑
        self.setCentralWidget(self.textEdit)                #显示编辑文字
        self.statusBar()                                    #建立状态栏

        ##建立动作
        #参数：获取图片，选项
        openFile = QAction(QIcon('web.png'), 'Open', self)
        openFile.setShortcut('Ctrl+O')                      #设置快捷键
        openFile.setStatusTip('Open new File')              #提示语
        openFile.triggered.connect(self.showDialog)         #建立信号连接槽

        #建立菜单栏
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')                 #增加菜单
        fileMenu.addAction(openFile)                        #增动作，参数为动作

        self.setGeometry(300, 300, 350, 300)
        self.setWindowTitle('File dialog')
        self.show()

        #建立函数，用于显示对话框
    def showDialog(self):
        #文件对话框，打开文件名字 参数：新对话框名字，初始地址
        #返回参数列表，fname[0]为选着的地址
        fname = QFileDialog.getOpenFileName(self, 'Open file', '/home')

        if fname[0]:
            #打开文件，用可读模式
            f = open(fname[0], 'r')

            #with运行文件，运行完后自动关闭
            with f:
                #读取内容
                data = f.read()
                #设置到编辑框
                self.textEdit.setText(data)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())