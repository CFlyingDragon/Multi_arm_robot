# KDL version 1
#!/usr/bin/env python
#-*- coding:utf-8 -*-
############################
#File Name: kdl_module.py
#Author: Wang
#Mail: wang19920419@hotmail.com
#Created Time:2017-09-09 10:51:39
############################

# import sys
# sys.path.insert(0, "/home/file/catkin_ws/src/Basic_math/hrl-kdl-indigo-devel/hrl_geom/src")
# sys.path.insert(0, "/home/wangxu/catkin_ws/src/Basic_math/hrl-kdl-indigo-devel/pykdl_utils/src")

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

robot = URDF.from_xml_file("/home/d/catkin_ws/src/robot_description/armc_description/urdf/armc_description.urdf")
tree = kdl_tree_from_urdf_model(robot)
print tree.getNrOfSegments()
chain = tree.getChain("base", "tool0")
print chain.getNrOfJoints()
kdl_kin = KDLKinematics(robot, "base_link", "ee_link", tree)
q = [0,0,0,0,0,0]
pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
q_ik = kdl_kin.inverse(pose, [0.1,0.1,-0.1,0.1,0.1,0.1]) # inverse kinematics
if q_ik is not None:
    pose_sol = kdl_kin.forward(q_ik) # should equal pose
#J = kdl_kin.jacobian(q)
print 'q:', q
print 'q_ik:', q_ik
print 'pose:', pose
if q_ik is not None:
    print 'pose_sol:', pose_sol
#print 'J:', J