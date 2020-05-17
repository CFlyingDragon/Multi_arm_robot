#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用于写示教函数，后期可能会加入一定的学习
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：初稿：2019.5.13

import numpy as np

#************************获取关节角度,依次写入文件*********************#
class teaching(object):
    def __init__(self,path):
        self.data_path = path
        self.qq_list = []

    #读取关节角
    def get_joint_position(self,qq):
        self.current_qq = qq
        self.qq_list.append(qq)

    #使用添加模式写入数据到指定文件
    def write_data(self):
        with open(self.data_path, 'w') as file_to_write:
            print ('开始读写')
            write_data = np.array(self.qq_list)
            [row, col] = write_data.shape
            for i in range(row):
                for j in range(col):
                    data = write_data[i, j]
                    file_to_write.write(str(data))
                    file_to_write.write(' ')  # 用空格分隔数据
                file_to_write.write('\n')  # 末尾转行符





