#-*-coding:utf-8-*-
#本文档用于读取文件和写入文件
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.2.20

import numpy as np

#读取数据	
def read(path):
	''' read 函数用于读取数据,数据由单空格隔开
		输入参数：path 读取文件的路径
		返回参数：数组，列表格式
	'''
	read_data = []
	with open(path, 'r') as file_to_read:
		while True:
			line = file_to_read.readline() # 整行读取数据
			if not line:
				break
				
			line=line.split(" ") #用空格拆分数据
			
			line = line[:-1]  #切片去除末尾转行符	
			read_data .append(line)
	read_data = list(read_data)		
	return read_data
	
#将数据写入文件，用单空格分隔数据，末尾转行
def write(write_data,path):
	''' write函数用于写入矩阵型数据，共两个参数
		第一个参数为所需写入的数据，第二格参数为写入文件路径和名字
		文件为用单空格分隔数据，末尾转行
		写入模式为添加模式
	'''
	with open(path, 'a') as file_to_write:
		print ('开始读写')
		write_data = np.array(write_data)
		[row,col] = write_data.shape
		for i in range(row):
			for j in range(col):
				data = write_data[i,j]
				file_to_write.write(str(data))
				file_to_write.write(' ') #用空格分隔数据
			file_to_write.write('\n') #末尾转行符
