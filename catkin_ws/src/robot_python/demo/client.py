#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints') #在客户端中我们不需要创建node，这是一种便捷方法，可以阻止名为add_two_ints的服务可用。
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)#创建一个用于调用服务的句柄，可以像调用函数一样，调用句柄
        resp1 = add_two_ints(x, y) #因为我们已经将服务的类型声明为AddTwoInts，所以它会为您生成AddTwoIntsRequest对象（可以自由传递）。
        return resp1.sum
    except rospy.ServiceException, e: #如果调用失败，可能会抛出rospy.ServiceException
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
