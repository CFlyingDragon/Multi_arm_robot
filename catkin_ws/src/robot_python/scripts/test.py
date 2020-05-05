#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于通讯测试
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2019.11.30

import roslib
import rospkg
import time
import numpy.linalg as nla
roslib.load_manifest('robot_python')
from robot_python import BaseFunction as bf
from robot_python import Kinematics as kin
from robot_python import RobotParameter as rp

from math import pi
import numpy as np
DH_0 = rp.DH0_armc
theta = DH_0[:,0]
alpha = DH_0[:,1]
a = DH_0[:,2]
d = DH_0[:,3]

qr = np.array([0,0,0,0,0,0,0])*pi/180
qrr = np.array([42,62,30,40,91,32,-1])*pi/180
current_psi = 0
succeed_label = 1
#J1 = kin.jacobian(DH_0,qr)
#J2 = kin.jeco_0(DH_0, qr)

#print np.around(J1, decimals=6)
#print np.around(J2, decimals=6)

psi_0 = 0.3
Te = kin.fkine(theta + qr,alpha,a,d)

print Te

time1 = time.clock()
qq = kin.iterate_ikine(DH_0, qrr, Te,)
#[qq,current_psi,succeed_label]= kin.aa_ikine(Te,psi_0,qrr)
#[qq,current_psi,succeed_label]= kin.armc_ikine(Te,psi_0,qrr)
time2 = time.clock()
Tn = kin.fkine(theta + qq,alpha,a,d)
delte =nla.norm(Te[0:3,3] - Tn[0:3,3])
qq = qq*180/3.14

print "计算所需时间： %s" % (time2 - time1)
print "关节角度：%s" % qq
print "求取关节臂型角：%s" % current_psi
print "判断关节角求其是否成功：%s" % succeed_label
psi = bf.arm_angle_by_joint(qq,DH_0)
print "通过关节角计算臂型角：%s" % psi
print "计算误差：%s" % delte

