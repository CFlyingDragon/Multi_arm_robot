#!/usr/bin/env python
#-*-coding:utf-8-*-
#本文档用于建立KDL库函数的基础运用
#程序员：陈永厅
#版权：哈尔滨工业大学
#日期：初稿：2020.1.16

import numpy as np

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

##ros 相关的转换
from tf_conversions import posemath
from geometry_msgs.msg import Pose

##自定义函数
import BaseFunction as bf

#================URDF建立机器人运动学求解类=================#
class MyKdlByUrdf(KDLKinematics):
    def __init__(self):
        print "建立机器人模型"
        # 建立机器人模型参数
        robot_urdf = URDF.from_xml_file(
            "/home/d/catkin_ws/src/robot_description/armc_description/urdf/armc_description.urdf")
        tree = kdl_tree_from_urdf_model(robot_urdf)
        super(MyKdlByUrdf,self).__init__(robot_urdf, "base_link", "sensor_link", tree)

#================DH建立机器人运动学求解类=================#
class MyKdlByDH(object):
    def __init__(self,DH):
        self.DH = DH
        ##在构造函数中建立机器人模型
        #DH(theta,alpha,a,d)
        n = len(DH[:,0])
        #建立机器人坐标，关节
        frms = []
        jnts = []
        for i in range(n):
            jnts.append(kdl.Joint(kdl.Joint.RotZ))
            #建立坐标系
            trans = bf.trans(DH[i,0],DH[i,1],DH[i,2],DH[i,3])
            vetor = kdl.Vector(trans[3,0], trans[3,1], trans[3,2])
            frm1 = kdl.Frame(kdl.Rotation.RotX(-np.pi / 2), kdl.Vector(0, 0, 239.5))


