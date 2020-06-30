#-*-coding:utf-8-*-
#本文档用于绘图
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.2.20  2.21 2.23
import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mal
from mpl_toolkits.mplot3d import Axes3D
from pylab import * 
from matplotlib.ticker import MultipleLocator, FormatStrFormatter  

#二维绘图,时间相同的多维变量
def plot2_nd(t,X,title='myplot',xlab='IVariable',ylab='DVariable'):
	'''
	:param t: 时间
	:param X: Rm*n维变量
	:param title:
	:param xlab:
	:param ylab:
	:return:
	'''
	#获取维数
	n = len(X[0, :])

	#绘图
	plt.figure()

	for i in range(n):
		string = "X" + str(i + 1)
		plt.plot(t, X[:, i],label=string)
	plt.title(title)
	plt.xlabel(xlab)
	plt.ylabel(ylab)
	plt.legend()
	plt.show()

#二维绘图，输入参数x,y,title,xlable,ylable
def plot2r(x1,x2,title='myplot',xlab='IVariable',ylab='DVariable'):
	'''
		本函数用于绘制2维图，总共5个输入参数,前两个为必需参数
		第一个参数为自变量x，第二个为因变量y
		后三个以此为标题、X标签、Y 标签，为字符串格式
	'''
	plt.figure()
	plt.plot(x1,label='wish',color = 'b')
	plt.plot(x2,label='genetic',color = 'r')
	plt.title(title)
	plt.xlabel(xlab)
	plt.ylabel(ylab)
	plt.legend()
	plt.show()

def plot2d(x,y,title='myplot',xlab='IVariable',ylab='DVariable'):
	'''
		本函数用于绘制2维图，总共5个输入参数,前两个为必需参数
		第一个参数为自变量x，第二个为因变量y
		后三个以此为标题、X标签、Y 标签，为字符串格式
	'''
	plt.figure()
	plt.plot(x,y)#,label='wish')
	plt.title(title)
	plt.xlabel(xlab)
	plt.ylabel(ylab)
	plt.legend()
	plt.show()
	
#三维绘图，输入x,y,z
def plot3d(x,y,z,tit="3D_Curve",xlab='x',ylab='y',zlab='z'):
	'''
		本函数用于绘制3维图，总共7个输入参数,前两个为必需参数
		第一个参数为自变量x，第二个为因变量y,第三个为因变量z
		后三个以此为标题、X标签、Y 标签，为字符串格式
	'''
	fig = plt.figure()
	mal.rcParams['legend.fontsize'] = 10
	ax = fig.gca(projection='3d')
	
	# 设置图片信息
	ax.set_title(tit)
	ax.set_xlabel(xlab)
	ax.set_ylabel(ylab)
	ax.set_zlabel(zlab)
	# 绘图
	figure = ax.plot(x, y, z)
	 
	ax.legend()
	plt.show()

#绘制三维点图，输入x,y,z
def plot3Point(x1,y1,z1,x2,y2,z2,tit='myplot',xlab='x',ylab='y',zlab='z'):
	'''
		本函数用于绘制3维点图
		第一个参数为自变量x，第二个为因变量y,第三个为因变量z
	'''
	fig = plt.figure()
	mpl.rcParams['legend.fontsize'] = 10
	ax = fig.gca(projection='3d')
	ax.plot(x1, y1, z1, label='wish')
	ax.scatter(x2, y2, z2, label='genetics',color = 'r')
	plt.title(tit,fontproperties='SimHei')
	plt.xlabel(xlab)
	plt.xlabel(ylab)
	plt.xlabel(zlab)
	ax.legend()
	ax.legend()
	plt.show()

