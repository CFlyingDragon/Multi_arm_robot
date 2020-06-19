#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于界面相关函数
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.4.26
#系统函数

import sys
from PyQt5.QtWidgets import QMainWindow, QWidget, QApplication
from PyQt5.QtCore import Qt


class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.initUI()

    def initUI(self):
        self.setWindowTitle("鼠标键盘事件示例")
        self.setCentralWidget(QWidget())  # 指定主窗口中心部件
        self.statusBar().showMessage("ready")  # 状态栏显示信息
        self.resize(300, 185)

    # 重新实现各事件处理程序
    def keyPressEvent(self, event):
        key = event.key()
        if Qt.Key_A <= key <= Qt.Key_Z:
            if event.modifiers() & Qt.ShiftModifier:  # Shift 键被按下
                self.statusBar().showMessage('"Shift+%s" pressed' % chr(key), 500)
            elif event.modifiers() & Qt.ControlModifier:  # Ctrl 键被按下
                self.statusBar().showMessage('"Control+%s" pressed' % chr(key), 500)
            elif event.modifiers() & Qt.AltModifier:  # Alt 键被按下
                self.statusBar().showMessage('"Alt+%s" pressed' % chr(key), 500)
            else:
                self.statusBar().showMessage('"%s" pressed' % chr(key), 500)

        elif key == Qt.Key_Home:
            self.statusBar().showMessage('"Home" pressed', 500)
        elif key == Qt.Key_End:
            self.statusBar().showMessage('"End" pressed', 500)
        elif key == Qt.Key_PageUp:
            self.statusBar().showMessage('"PageUp" pressed', 500)
        elif key == Qt.Key_PageDown:
            self.statusBar().showMessage('"PageDown" pressed', 500)
        else:  # 其它未设定的情况
            QWidget.keyPressEvent(self, event)  # 留给基类处理
        '''
        其它常用按键：
        Qt.Key_Escape,Qt.Key_Tab,Qt.Key_Backspace,Qt.Key_Return,Qt.Key_Enter,
        Qt.Key_Insert,Qt.Key_Delete,Qt.Key_Pause,Qt.Key_Print,Qt.Key_F1...Qt.Key_F12,
        Qt.Key_Space,Qt.Key_0...Qt.Key_9,Qt.Key_Colon,Qt.Key_Semicolon,Qt.Key_Equal
        ...
        '''

    def mousePressEvent(self, event):  # 鼠标按下事件
        pos = event.pos()  # 返回鼠标所在点QPoint
        self.statusBar().showMessage('Mouse is pressed at (%d,%d) of widget ' % (pos.x(), pos.y()), 500)
        globalPos = self.mapToGlobal(pos)
        print('Mouse is pressed at (%d,%d) of screen ' % (globalPos.x(), globalPos.y()))

    def mouseReleaseEvent(self, event):  # 鼠标释放事件
        pos = event.pos()  # 返回鼠标所在点QPoint
        self.statusBar().showMessage('Mouse is released at (%d,%d) of widget ' % (pos.x(), pos.y()), 500)
        if event.button() == Qt.LeftButton:
            print("左键")
        elif event.button() == Qt.MidButton:
            print("中键")
        elif event.button() == Qt.RightButton:
            print("右键")

    def mouseDoubleClickEvent(self, event):  # 鼠标双击事件
        pos = event.pos()  # 返回鼠标所在点QPoint
        self.statusBar().showMessage('Mouse is double-clicked at (%d,%d) of widget ' % (pos.x(), pos.y()), 500)

    def mouseMoveEvent(self, event):  # 鼠标移动事件
        pos = event.pos()  # 返回鼠标所在点QPoint
        self.statusBar().showMessage('Mouse is moving at (%d,%d) of widget ' % (pos.x(), pos.y()), 500)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    mw = MainWindow()
    mw.show()
    sys.exit(app.exec_())
