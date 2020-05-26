#-*-coding:utf-8-*-
#!/usr/bin/env python
#本文档用于numpy学习
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳）
#日期：2020.5.20
#导入numpy模块
import numpy as np
import numpy.linalg as nla

#创建元素为0的数组
shape1 = 10
shape2 = [2, 3]
shape4 = [4, 5, 6, 7]
a1 = np.zeros(shape1, dtype=float)
a2 = np.zeros(shape2)
a3 = np.zeros(shape4, dtype=int)

#创建元素为1的数组
b2 = np.ones(shape2)

#创建对角矩阵，对角线为1
c2 = np.eye(shape1)

#nd.array转换数据
a_list = list([1,2,3,4,5,6,7])
a_array = np.array(a_list)
print a_list

#Order 'C'为按行的 C 风格数组，'F'为按列的 Fortran 风格数组
#建立行向量
v_row = np.zeros(10, order= 'C')
print v_row
#建立列向量
v_col = np.zeros(10, order= 'F')
print v_col