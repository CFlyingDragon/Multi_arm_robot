# -*- coding: utf-8 -*-

"""
PyQt5 tutorial

This is a simple drag and
drop example.

author: py40.com
last edited: 2017年3月
"""
import sys
from PyQt5.QtWidgets import (QPushButton, QWidget,
                             QLineEdit, QApplication)

#自定义按钮，继承于按钮
class Button(QPushButton):
    def __init__(self, title, parent):
        super(QPushButton, self).__init__(title, parent)

        self.setAcceptDrops(True)

    def dragEnterEvent(self, e):

        if e.mimeData().hasFormat('text/plain'):
            e.accept()
        else:
            e.ignore()

    def dropEvent(self, e):

        self.setText(e.mimeData().text())


class Example(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.initUI()

    def initUI(self):
        #线编辑
        edit = QLineEdit('', self)
        #设置拖拽
        edit.setDragEnabled(True)
        edit.move(30, 65)

        button = Button("Button", self)
        button.move(190, 65)

        self.setWindowTitle('Simple drag & drop')
        self.setGeometry(300, 300, 300, 150)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    ex.show()
    app.exec_()