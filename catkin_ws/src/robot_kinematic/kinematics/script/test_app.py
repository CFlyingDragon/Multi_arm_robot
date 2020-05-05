#-*-coding:utf-8-*-
#!/usr/bin/env python
#本文档用于测试7自由度机械臂的运动学
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.9.20
import numpy as np
import math

#自定义函数模块
import Kinematics as kin
import PathPlan as pap
from RobotParameter import DH0_armc as DH_0
import EndPoint as enp

#ros相关模块
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

#获得DH参数
theta0 = DH_0[:, 0]; alpha = DH_0[:, 1]; a = DH_0[:, 2]; d = DH_0[:, 3]

#=================规划轨迹测试模块=================#
##圆轨迹测试
rc = 100
[qr_init,X0_e,T] = enp.circlePoint_armc(rc)

##直线轨迹测试
#l = 100
#[qr_init,X0_e,T] = enp.multipointLine_armc(l)

#位置级求逆函数测试测试
[qq,qv,qa] = pap.multipoint_plan_position(qr_init,X0_e,T)

def talker(Q):
    '''
    param Q:输入关节角度
    return:关节角度
    '''
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('test_state_publisher', anonymous=True)
    rate = rospy.Rate(100)  # 10hz
    joint_state = JointState()

    k_max = len(Q[0, :]) - 1
    k = 0
    while not rospy.is_shutdown():
        if k > k_max:
            break
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "Joint7"]
        joint_state.position = [Q[0, k], Q[1, k], Q[2, k], Q[3, k], Q[4, k], Q[5, k], Q[6, k]]

        rospy.loginfo("发送")
        rospy.loginfo(Q[:, k])

        pub.publish(joint_state)
        rate.sleep()
        k = k + 1

if __name__ == '__main__':
    try:
        talker(qq)
    except rospy.ROSInterruptException:
        pass
