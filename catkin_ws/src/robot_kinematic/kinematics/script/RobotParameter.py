#-*-coding:utf-8-*-
#!/usr/bin/env python
#本文档用于存储机器人的参数
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：2019.10.1
import numpy as np
import math
from math import pi

#自制机械臂armc的DH参数表,长度单位m
DH0_armc = np.array([[0, -pi/2, 0, 0.248],
					[0,  pi/2, 0, 0    ],
					[0, -pi/2, 0, 0.305],
					[0,  pi/2, 0, 0    ],
					[0, -pi/2, 0, 0.306],
					[0,  pi/2, 0, 0    ],
					[0,  0,    0, 0.190]])

#自制机械臂armc的关节极限，单位为rad
q_min_armc = np.array([-200,-90,-200,-90,-200,-90,-200])*pi/180
q_max_armc = np.array([200,90,200,90,200,90,200])*pi/180
q_limit_armc = np.array([[-200,-90,-200,-90,-200,-90,-200],
						 [ 200, 90, 200, 90, 200, 90, 200]])*pi/180
