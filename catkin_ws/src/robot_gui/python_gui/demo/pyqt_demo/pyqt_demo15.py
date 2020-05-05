# -*- coding: utf-8 -*-

"""
PyQt5 tutorial

This example shows how to use
a QComboBox widget.

author: py40.com
last edited: 2017年3月
"""
import sys
from PyQt5.QtWidgets import (QWidget, QLabel,
                             QComboBox, QApplication)


class Example(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.initUI()

    def initUI(self):
        #建立标签
        self.lbl = QLabel("Ubuntu", self)
        #建立下拉列表
        combo = QComboBox(self)
        combo.addItem("Ubuntu")
        combo.addItem("Mandriva")
        combo.addItem("Fedora")
        combo.addItem("Arch")
        combo.addItem("Gentoo")

        combo.move(50, 50)
        self.lbl.move(50, 150)

        #下拉列表信号槽
        combo.activated[str].connect(self.onActivated)

        self.setGeometry(300, 300, 300, 200)
        self.setWindowTitle('QComboBox')
        self.show()

    def onActivated(self, text):
        #设置文字
        self.lbl.setText(text)
        #调整尺寸
        self.lbl.adjustSize()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())