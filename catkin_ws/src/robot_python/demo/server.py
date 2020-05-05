#!/usr/bin/env python
from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b) #因为我们已经将服务的类型声明为AddTwoInts，所以它会为您生成AddTwoIntsRequest对象（可以自由传递）。

def add_two_ints_server():
    rospy.init_node('add_two_ints_server') #声明节点为add_two_ints_server
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)#使用AddTwoInts（之前教程中建立的）服务类型声明一个名为add_two_ints的新服务。所有请求都传递给handle_add_two_ints函数。
    print "Ready to add two ints."
    rospy.spin() #此语句保证直到节点被关闭，代码才会停止运行

if __name__ == "__main__":
    add_two_ints_server()
