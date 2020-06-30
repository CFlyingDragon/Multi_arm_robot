#!/usr/bin/env python
#-*- coding:utf-8 -*-

import math

import PyKDL

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

robot = URDF.from_xml_file("/home/d/catkin_ws/src/robot_description/armc_description/urdf/armc_description.urdf")
tree = kdl_tree_from_urdf_model(robot)

print tree.getNrOfSegments()
chain = tree.getChain("base_link", "sensor_link")
print chain.getNrOfSegments()
print chain.getNrOfJoints()

#正运动学
fk = PyKDL.ChainFkSolverPos_recursive(chain)

pos = PyKDL.Frame()
q = PyKDL.JntArray(7)
for i in range(7):
    q[i] = 0
q[0] = 1
fk_flag = fk.JntToCart(q, pos)
print "fk_flag", fk_flag
print "pos", pos

#逆运动学
ik_v = PyKDL.ChainIkSolverVel_pinv(chain)
ik = PyKDL.ChainIkSolverPos_NR(chain, fk, ik_v, maxiter=100, eps=math.pow(10, -9))

qq = PyKDL.JntArray(7)
qq_k = PyKDL.JntArray(7)
ik.CartToJnt(qq_k, pos, qq)
print "qq:", qq