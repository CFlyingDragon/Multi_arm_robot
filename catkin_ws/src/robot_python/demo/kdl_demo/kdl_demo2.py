#!/usr/bin/python
#-*-coding:utf-8-*-
#研究运动树和运动学链

import PyKDL
#********创建段*******#
#坐标
frame1 = PyKDL.Frame(PyKDL.Rotation.RPY(0, 1, 0), PyKDL.Vector(3, 2, 4))
print "frame1:\n", frame1
#关节
joint1 = PyKDL.Joint(PyKDL.Joint.RotZ)
print "joint:", joint1

#惯性张量
m = 0.5
oc = PyKDL.Vector(1, 0, 1)
Ic = PyKDL.RotationalInertia(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

RBI = PyKDL.RigidBodyInertia(m, oc, Ic)

m = RBI .getMass()
print "m:\n", m

oc = RBI.getCOG()
print "oc:\n", oc

#创建段
segment1 = PyKDL.Segment(joint1, frame1, RBI)
print "segment1:\n", segment1

#********创建链*******#
links = []
for i in range(6):
    links.append(segment1)

chain1 = PyKDL.Chain()  # 建立机器人对象
for i in range(6):
    chain1.addSegment(links[i])

joint_num = chain1.getNrOfJoints()

print "joint_num:", joint_num
Segment_num = chain1.getNrOfSegments()
print "Segment_num:", Segment_num

Segment_6 = chain1.getSegment(5)
print "Segment_6:", Segment_6

#*******创建树******#
#建立树
tree = PyKDL.Tree("my_robot")

#增加segment
for i in range(6):
    str_i = "link" + str(i)
    tree.addSegment(segment1, str_i)

#显示信息
chain2 = tree.getChain("link0", "link3")
joint_num = chain2.getNrOfJoints()

print "joint_num:", joint_num
Segment_num = chain2.getNrOfSegments()
print "Segment_num:", Segment_num

Segment_6 = chain2.getSegment(5)
print "Segment_6:", Segment_6