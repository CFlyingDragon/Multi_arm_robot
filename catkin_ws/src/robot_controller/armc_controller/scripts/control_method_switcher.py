#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档为客客户端，请求切换控制方法
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.12.11
import sys
import rospy
from armc_controller.srv import SetArmcConfigure
from controller_manager_msgs.srv import SwitchController

def controller_client():
    rospy.wait_for_service("controller_manager/switch_controller")  # 在客户端中我们不需要创建node，这是一种便捷方法，可以阻止名为"/set_control_method"的服务可用。
    try:
        control_method = "Joint Cyclic Position"
        set_switch = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)  # 创建一个用于调用服务的句柄，可以像调用函数一样，调用句柄
        print set_switch
        resp = set_switch(control_method)
        print resp.result
        if (resp.result):
            print "控制方法成功设置为：%s" % control_method
        else:
            print"设置失败！"

    except rospy.ServiceException, e:  # 如果调用失败，可能会抛出rospy.ServiceException
        print "Service call failed: %s" % e

def method_client():
    rospy.wait_for_service("/set_control_method") #在客户端中我们不需要创建node，这是一种便捷方法，可以阻止名为"/set_control_method"的服务可用。
    try:
        control_method = "Joint Cyclic Position"
        set_method = rospy.ServiceProxy("/set_control_method", SetArmcConfigure)#创建一个用于调用服务的句柄，可以像调用函数一样，调用句柄
        print set_method
        resp = set_method(control_method)
        print resp.result
        if(resp.result):
            print "控制方法成功设置为：%s" % control_method
        else:
            print"设置失败！"

    except rospy.ServiceException, e: #如果调用失败，可能会抛出rospy.ServiceException
        print "Service call failed: %s"%e

if __name__ == "__main__":
    controller_client()