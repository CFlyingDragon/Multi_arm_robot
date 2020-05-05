#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于读取文件和写入文件
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2019.4.21
import sys, cv2, time
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from robotsgui import *

#自定义文件


class MyWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MyWindow, self).__init__(parent)
        self.setupUi(self)
        self.initUI()
    def initUI(self):
        #sld.valueChanged.connect(lcd.display)
        #创建滑条
        self.verticalSlider.valueChanged.connect(self.lcdNumber.display)
        # ======================菜单栏功能模块=======================#
        #创建菜单
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        gazeboMenu = menubar.addMenu('&gazebo')
        controlMenu = menubar.addMenu('&control')
        testMenu = menubar.addMenu('&test')
        helpMenu = menubar.addMenu('&help')

        #文件菜单中打开文件操作
        openFile = QAction(QIcon('exit.png'), 'Open', self)
        openFile.setShortcut('Ctrl+O')
        openFile.setStatusTip('Open new File')
        openFile.triggered.connect(self.fileOpen)
        fileMenu.addAction(openFile)

        #文件菜单中关闭操作
        exitAction = QAction(QIcon('exit.png'), '&Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.setStatusTip('Exit application')
        exitAction.triggered.connect(qApp.quit)
        fileMenu.addAction(exitAction)

        # =======================按钮功能模块=======================#
        #读取积分自适应阻抗参数MBKI
        self.readButton.clicked.connect(self.getMBKI)
        self.readDirButton.clicked.connect(self.getPubDir)
        self.pubButton.clicked.connect(self.pubtopic)

    #打开文件的地址和内容
    def fileOpen(self):
        #打开文件操作
        fname = QFileDialog.getOpenFileName(self, 'Open file', '/home')

        if fname[0]:
            f = open(fname[0], 'r')
            self.filedir = fname[0]

            with f:
                data = f.read()
                self.pubdata = data
                self.textEdit.setText(data)

    #===============按钮功能模块相关函数================#
    #读取阻抗参数
    def getMBKI(self):

        self.imc_m = self.lineEdit_m.text()
        self.imc_b = self.lineEdit_b.text()
        self.imc_k = self.lineEdit_k.text()
        self.imc_i = self.lineEdit_i.text()
        msg = "M:" + str(self.imc_m) + "\n"\
                + "B:" + str(self.imc_b) + "\n" \
                    + "K:" + str(self.imc_k) + "\n" \
                        + "I:" + str(self.imc_i)

        self.textEdit.setText(msg)

    #读取话题发送地址
    def getPubDir(self):
        self.pubdir = self.lineEdit_dir.text()
        self.textEdit.setText(self.pubdir)

    #发送话题
    def pubtopic(self):
        pubstr = "run pubulisher topic:" + str(self.pubdir)
        self.textEdit.setText(pubstr)

        #视频处理
    def videoprocessing(self):
        print("gogo")
        global videoName  # 在这里设置全局变量以便在线程中使用
        videoName, videoType = QFileDialog.getOpenFileName(self,
                               "打开视频",
                               "",
                               " *.mp4;;*.avi;;All Files (*)")
        print videoName
        print videoType
        th = Thread(self)
        th.changePixmap.connect(self.setImage)  #连接图像处理
        th.start()

    def setImage(self, image):
        self.label_v.setPixmap(QPixmap.fromImage(image))

    #处理图片函数
    def imageprocessing(self):
        print("hehe")
        imgName, imgType = QFileDialog.getOpenFileName(self,
                                                       "打开图片",
                                                       "",
                                                       " *.jpg;;*.png;;*.jpeg;;*.bmp;;All Files (*)")

        # 适应设计label时的大小
        png = QtGui.QPixmap(imgName).scaled(self.label_i.width(), self.label_i.height())
        self.label_i.setPixmap(png)

class Thread(QThread):  # 采用线程来播放视频

    changePixmap = pyqtSignal(QtGui.QImage)

    #重写运行
    def run(self):
        #打开图形文件
        cap = cv2.VideoCapture(videoName)
        print(videoName)
        while (cap.isOpened() == True):
            ret, frame = cap.read()
            if ret:
                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                convertToQtFormat = QtGui.QImage(rgbImage.data, rgbImage.shape[1], rgbImage.shape[0],
                                                 QImage.Format_RGB888)  # 在这里可以对每帧图像进行处理，
                p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)
                time.sleep(0.01)  # 控制视频播放的速度
            else:
                break

if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = MyWindow()
    myWin.show()
    sys.exit(app.exec_())

