#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于界面图像相关的函数
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.4.27
from PyQt5.QtWidgets import QApplication,QMainWindow,QGridLayout
from PyQt5.QtCore import QTimer
import sys,time
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
import matplotlib
import matplotlib.cbook as cbook

#=======================主窗口相关图像处理函数==================#
#-----------定义画板------------#
class Figure_Canvas(FigureCanvas):
    def __init__(self,parent=None,width=3.9,height=2.7,dpi=100):
        self.fig=Figure(figsize=(width,height),dpi=100)
        super(Figure_Canvas,self).__init__(self.fig)
        self.ax=self.fig.add_subplot(111)
    def test(self):
        x=[1,2,3,4,5,6,7]
        y=[2,1,3,5,6,4,3]
        self.ax.plot(x,y)

#-----------定义滑板------------#