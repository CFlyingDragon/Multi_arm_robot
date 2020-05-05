#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档为客客户端，请求切换控制方法
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.12.25
import rospy
from armc_controller.srv import SetArmcConfigure

def method_test(req):
    print req
    print "get control method is : %s" % req
    return True

def method_server():
    rospy.init_node('set_control_method') #声明节点为add_two_ints_server
    s = rospy.Service('/set_control_method', SetArmcConfigure, method_test)#使用AddTwoInts（之前教程中建立的）服务类型声明一个名为add_two_ints的新服务。所有请求都传递给handle_add_two_ints函数。
    print "Ready to get control method!"
    rospy.spin() #此语句保证直到节点被关闭，代码才会停止运行

if __name__ == "__main__":
    method_server()