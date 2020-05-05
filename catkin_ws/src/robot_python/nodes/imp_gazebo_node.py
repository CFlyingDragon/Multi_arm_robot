#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于上层导纳控制仿真
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.1.11
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

import threading
import time
import numpy as np

#自定义函数
from robot_python import ImpedanceControl as imc

##定义全局变量
F = np.zeros([3,6])
qq = np.zeros(7)
omega_k = np.zeros(6)
##关节角订阅函数
def joint_callback(msg):
    global qq
    for i in range(7):
        qq[i] = msg.position[i]

##六维力回调函数
def force_callback(msg):
    global F
    Fe = np.zeros(6)
    Fe[0] = msg.wrench.force.x
    Fe[1] = msg.wrench.force.y
    Fe[2] = msg.wrench.force.z
    Fe[3] = msg.wrench.torque.x
    Fe[4] = msg.wrench.torque.y
    Fe[5] = msg.wrench.torque.z

    base_F = imc.force_end_to_base(qq, Fe)
    F[0,:] = F[1, :]
    F[1,:] = F[2, :]
    F[2,:] = base_F

##末端力订阅线程
def thread_spin():
    rospy.spin()


##阻抗控制运行节点
def node_cloop(pub):

    ##控制参数
    global F                  #当前3时刻位置偏差，全局变量
    global qq                 #当前关节角位置
    global omega_k

    E = np.zeros([2, 6])       #上2时刻位置位置偏差
    T = 0.01
    Fd = np.zeros(6)

    [Xd, qq_init] = imc.get_expect_pos()      #末端期望位姿,可放入循环中
    [Md, Bd, Kd] = imc.get_imc_parameter()    #可放入循环中，实时改变参数

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        #hello_str = "Current time is %s" % rospy.get_time()
        #print hello_str
        print "误差计算E：%s" % E[1, :]
        current_E = imc.get_current_error(Md, Bd, Kd, T, F, E)
        print "误差计算E：%s" % E[1, :]
        print omega_k
        omega_kk = imc.force_adaption(Kd, Fd, F[-1, :], omega_k)
        omega_k = omega_kk

        print "当前末端力：%s" % F[2,:]
        print "当前关节角：%s" % qq
        print "误差计算：%s" % current_E[1,:]
        print "误差计算E：%s" % E[1,:]
        qr = imc.get_current_joint(Xd, E[1, :], qq_init, omega_kk)
        command_data = Float64MultiArray()
        command_data.data = qr
        pub.publish(command_data)
        E = np.copy(current_E)
        rate.sleep()

def main():
    rospy.init_node('ImpendanceController_node')
    rospy.Subscriber('/robot1/joint_states', JointState, joint_callback)
    rospy.Subscriber('/robot1/ft_sensor_topic', WrenchStamped, force_callback)
    pub = rospy.Publisher('/robot1/armc_position_controller/command', Float64MultiArray, queue_size=10)

    t1 = threading.Thread(target=thread_spin)   # 末端位置订阅线程

    print "Impendance controller begain run!;"
    t1.start()
    node_cloop(pub)

if __name__ == '__main__':
    main()
