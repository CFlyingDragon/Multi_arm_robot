# -*- coding: utf-8 -*-

"""
PyQt5 tutorial

In this example, we draw text in Russian azbuka.

author: py40.com
last edited: 2017年3月
"""
import sys
from PyQt5.QtWidgets import QWidget, QApplication
from PyQt5.QtGui import QPainter, QColor, QFont
from PyQt5.QtCore import Qt


class Example(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.initUI()

    def initUI(self):
        self.text = u'\u041b\u0435\u0432 \u041d\u0438\u043a\u043e\u043b\u0430\
\u0435\u0432\u0438\u0447 \u0422\u043e\u043b\u0441\u0442\u043e\u0439: \n\
\u0410\u043d\u043d\u0430 \u041a\u0430\u0440\u0435\u043d\u0438\u043d\u0430'

        self.setGeometry(300, 300, 280, 170)
        self.setWindowTitle('Draw text')
        self.show()

    #函数重载
    def paintEvent(self, event):
        qp = QPainter()                    #绘画
        qp.begin(self)                     #开始
        self.drawText(event, qp)
        qp.end()                           #结束

    #设置文本
    def drawText(self, event, qp):
        qp.setPen(QColor(168, 34, 3))                          #设置笔的颜色
        qp.setFont(QFont('Decorative', 10))                    #设置字体
        qp.drawText(event.rect(), Qt.AlignCenter, self.text)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())