#!/usr/bin/env python
#-*-coding:utf-8-*-
'''
	功能：通过moveit API驱动机械臂,本程序通过python写一个样板程序
	作者：飞龙（CYT)
	时间：2019.8.19
'''
##第一步，导入相关功能包
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

###========================准备工作==========================###
##初始化moveit_commander和rospy
print "=============Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveit_group_python_interface_tutorial",anonymous=True)

##实例化ROBOTCommander对象，这个接口机器人总入口
robot = moveit_commander.RobotCommander()

##实例化PlanningSceneInterface对象，这个接口与机器人世界有关
scene = moveit_commander.PlanningSceneInterface()

##实例化MoveitGroupCommander对象，这个接口应用于一组关节
group = moveit_commander.MoveGroupCommander('armc')

##创建DisplayTrajectory发布器，可以实现轨迹在rivz中实现显示
display_trajectory_publisher = rospy.Publisher(
						'/move_group/display_planned_path',
						moveit_msgs.msg.DisplayTrajectory)
##等待Rviz初始化
print "==========Waiting for RVIZ..."
rospy.sleep(5)
print "==========Starting tutorial"

###====================获得基本信息=======================###
##打印参考系的名称
print "=============Reference frame: %s" % group.get_planning_frame()

##打印这个组末端执行器的名称
print "===========End_effector_link: %s" % group.get_end_effector_link()

##获得机器人的组
print "============Robot Groups:"
print robot.get_group_names()

##用于调试，打印机器人的状态
print "===========Printing robot state"
print robot.get_current_state()
print "==========="

###================笛卡尔空间设置姿态目标=======================###
##设置一组期望位置，实现实现末端执行器到达期望的姿态目标
print "=============Generating plan1"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.5
pose_target.position.y = 0.05
pose_target.position.z = 0.6
group.set_pose_target(pose_target)

##调用规划器规划路径并在rivz中显示
plan1 = group.plan()
print "==============Waiting while RVIZ displays plan1 ...."
rospy.sleep(5)

##可以要求Rivz显示规划的轨迹，但group.plan()会自动显示
print "============Visualizing plan1"
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory)

print "===========Waiting while plan1 is visualized (again) ..."
rospy.sleep(5)

##移动到姿态目标
#go()是一个阻塞函数，需要一个控制器是激活的，执行后报告成功的轨迹
#Uncoment below line when working with a real robot
group.go(wait= True)

group.execute(plan1,wait = True)

###===============关节空间规划姿态目标================###
##设置连接空间目标并移动，首先清除目标姿态
group.clear_pose_targets()

##获取当前关节角度值
group_variable_values = group.get_current_joint_values()
print "============Joint values:",group_variable_values

##修改一个关节，规划移动到新的关节空间目标
group_variable_values[0] = 1.0
group.set_joint_value_target(group_variable_values)

plan2 = group.plan()

group.go(wait=True)

group.execute(plan2,wait = True)

print "=============Waiting while RVIZ displays plan2..."
rospy.sleep(5)

###=============笛卡尔路径规划==============####
##可以规划一个笛卡尔路径，通过直接为末端执行器指定航点列表
waypoints = []

#start with the current pose
waypoints.append(group.get_current_pose().pose)

#first orient grapper and move forward(+X)
wpose = geometry_msgs.msg.Pose()
wpose.orientation.w = 1.0
wpose.position.x = waypoints[0].position.x + 0.1
wpose.position.y = waypoints[0].position.y
wpose.position.z = waypoints[0].position.z
waypoints.append(copy.deepcopy(wpose))

#second move down
wpose.position.z -= 0.10
waypoints.append(copy.deepcopy(wpose))

#third move to the side
wpose.position.y += 0.05
waypoints.append(copy.deepcopy(wpose))

#建立规划组，第一个参数轨迹点，第二个参数笛卡尔转化最大值，第三个跳跃值，0为禁用
(plan3,fraction) = group.compute_cartesian_path(
						waypoints,  #waypoints to follow
						0.01,       #eef_step
						0.0)        #jump_threshold

#执行轨迹
group.execute(plan3,wait = True)

print "==============Waiting while RVIZ displanys plan3 ..."
rospy.sleep(5)

###==============添加/删除和附着/分离对象======================###
##定义碰撞对象消息
collision_object = moveit_msgs.msg.CollisionObject()

##完成后关闭
moveit_commander.roscpp_shutdown()












