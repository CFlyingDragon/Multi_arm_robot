#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于求7自由度机械臂armc基于moveit的圆规划
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.9.23
import numpy as np
import math
from math import pi
import sys
import copy

#ROS相关模块
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import PyKDL       #坐标变化有关的库
from geometry_msgs.msg import Pose
import tf.transformations as tr
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

#导入本地函数
import armc_moveit_plan as amp
from RobotParameter import DH0_armc as DH_0

import EndPoint as enp

def main():
	try:
		print "============ 按’Enter’连接到规划组....."
		raw_input()                 
		armc_moveit = amp.MoveGroupPythonIntefaceArmc()
		  
		##运动到初始位置
		print "============ 按’Enter’运动到初始位置....."
		raw_input()
		q_init = np.array([0,-30, 0, 90, 0 , 30 , 0])*(np.pi/180)
		armc_moveit.go_to_joint_state(q_init)
		
		#规划圆轨迹，将位置点存如数组中
		print "============ 按’Enter’将末端位置点存入....."
		raw_input()
		rc = 100
		[qr_init,X0_e,T] = enp.circlePoint_armc(rc)
		##直线轨迹测试
		#l = 100
		#[qr_init, X0_e, T] = enp.multipointLine_armc(l)

		#运动到一个点
		#armc_moveit.go_to_pose_goal()


		#将点加入轨迹列表中
		wpose = Pose()
		waypoints = []
		for i in range(len(X0_e[0,:])):
			quaternion = tr.quaternion_from_euler(X0_e[3,i],X0_e[4,i],X0_e[5,i])
			wpose.position.x = X0_e[0, i]
			wpose.position.y = X0_e[1, i]
			wpose.position.z = X0_e[2, i]
			wpose.orientation.x = quaternion[0]
			wpose.orientation.y = quaternion[1]
			wpose.orientation.z = quaternion[2]
			wpose.orientation.w = quaternion[3]
			waypoints.append(copy.deepcopy(wpose))
		
		#计算规划轨迹
		print "============ 按’Enter’计算圆轨迹点位姿....."
		raw_input()
		[plan, fraction_] = armc_moveit.compute_cartesian_path(waypoints)
		
		#显示轨迹
		print "============ 按’Enter’显示末端轨迹点....."
		raw_input()
		armc_moveit.display_trajectory(plan)
		 
		#执行轨迹
		print "============ 按’Enter’执行规划数据....."
		raw_input()
		armc_moveit.execute_plan(plan)

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

#运行主函数
if __name__ == '__main__':
  main()
