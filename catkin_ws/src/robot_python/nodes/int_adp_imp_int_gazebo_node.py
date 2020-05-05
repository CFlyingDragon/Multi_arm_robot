#!/usr/bin/env python
# -*-coding:utf-8-*-
#本文档用于上层积分自适应阻抗控制仿真,采用一阶积分求取
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
from robot_python import integral_adaption_impedance_controller_int as ia_imc

##定义全局变量
F = np.zeros(6)
qq = np.zeros(7)

##关节角订阅函数
def joint_callback(msg):
    global qq
    global X
    global n
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

    #对力进行坐标变换，同时更新最新时刻的力
    F = imc.force_end_to_base(qq, Fe)

##末端力订阅线程
def thread_spin():
    rospy.spin()

##阻抗控制运行节点
def node_cloop(pub):

    ##控制参数
    global F                  #当前3时刻位置偏差，全局变量
    global qq                 #当前关节角位置

    omega = np.zeros(6)       #积分项变量
    T = 0.01                  #时间周期

    Fd = np.zeros(6)
    Fd[0] = 2

    [Xd,Xvd,Xad,qq_init] = ia_imc.get_init_expect_pos()      #末端期望位姿,可放入循环中
    delta_F = F - Fd
    Xr = Xd
    Xvr = Xvd
    qq_guess = np.copy(qq_init)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        #调用积分自适应阻抗实时计算修正修正值
        delta_F = F - Fd
        [Xr_c, Xvr_c, omega_c] = ia_imc.int_adp_imc_int_eq(T,Xd,Xvd,Xad,delta_F,Xr,Xvr,omega)  #通过引用改变内部参数

        # print "当前末端力：%s" % F
        # print "当前关节角：%s" % qq
        print "末端参考位置：%s" % Xr

        qr = imc.get_control_joint(Xr_c, qq_guess)
        command_data = Float64MultiArray()
        command_data.data = qr
        #print "当前command关节角：%s" % qr
        pub.publish(command_data)
        Xr = Xr_c
        Xvr = Xvr_c
        omega = omega_c
        rate.sleep()

def main():
    rospy.init_node('ImpendanceController_node')
    rospy.Subscriber('/robot1/joint_states', JointState, joint_callback)
    rospy.Subscriber('/robot1/ft_sensor_topic', WrenchStamped, force_callback)
    pub = rospy.Publisher('/robot1/armc_position_controller/command', Float64MultiArray, queue_size=10)

    #建立用于引用的变量
    t1 = threading.Thread(target=thread_spin)   # 末端位置订阅线程

    print "Impendance controller begain run!;"
    t1.start()
    node_cloop(pub)

if __name__ == '__main__':
    main()
