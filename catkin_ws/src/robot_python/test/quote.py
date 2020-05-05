#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于测试python引用机制
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020/1/12
import numpy as np

X = np.zeros([4,4])

def data(X):
    '''
    :param X:
    :return:
    '''
    XX = np.copy(X)
    XX[0,1] = 1
    XXX = X
    XXX[3,3] = 6

def data_row(X_row):
    '''
    :param X_row:
    :return:
    '''
    X_row[2] = 2

def data_col(X_col):
    '''
    :param X_row:
    :return:
    '''
    X_col[3] = 3

print "传入X： %s" % X
data(X)
print "输出X： %s" % X

data_row(X[0,:])
print "输出X_row： %s" % X

data_col(X[:,2])
print "输出X_col： %s" % X

xx = 3*4+5\
    +4
print xx

def add(x):
    x = x + np.ones(2)
    print "now x = %s" % x
a = np.zeros(2)
for i in range(1000):
    add(a)