#!/usr/bin/python
#-*-coding:utf-8-*-
#本文档用测试python写服务器
#程序员：陈永厅
#版权：哈尔滨工业大学(深圳)
#日期：2020.9.19

import rospy
from robot_msg.srv import *

def get_visual(x):
    rospy.wait_for_service('visual_inspection')
    try:
        # create a handle to the visual_inspection service
        visual = rospy.ServiceProxy('visual_inspection', VisualVar)

        # simplified style
        resp1 = visual(x)
        print "resp1:", resp1

        # # formal style
        # resp2 = visual.call(VisualVarRequest(x))
        # print "resp2:", resp2

        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    x = 1
    if(x==1):
        print "检测门把手！"
    else:
        print "检测密码锁！"
    res = get_visual(x)
    if(res.flag):
        print "检测成功！"
        print "获得12数组：\n", res.T
    else:
        print "检测失败"

