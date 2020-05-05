# coding=utf-8
import numpy as np
import PyKDL as kdl

# import copy #用于深度拷贝

# 注意一些基本概念
# Segment初始化由绕的轴和末端姿态来生成，可能跟书上的D-H方法有点不一样，区别在于用末端和轴来定义一个关节
#   轴是在当前关节的坐标系中绕轴V旋转，默认设置为kdl.Joint.RotZ
#   f_tip，末端在当前坐标系中的姿态

jnts = []
frms = []

for i in range(6):
    jnts.append(kdl.Joint(kdl.Joint.RotZ))

# 注意，这个框架是f_tip，末端
frm1 = kdl.Frame(kdl.Rotation.RotX(-np.pi / 2), kdl.Vector(0, 0, 239.5))
frm2 = kdl.Frame(kdl.Rotation.RotX(0), kdl.Vector(250, 0, 0))
frm3 = kdl.Frame(kdl.Rotation.RotX(np.pi / 2), kdl.Vector(0, 262, 0))
frm4 = kdl.Frame(kdl.Rotation.RotX(-np.pi / 2), kdl.Vector(0, 0, 0))
frm5 = kdl.Frame(kdl.Rotation.RotX(np.pi / 2), kdl.Vector(0, 0, 0))
frm6 = kdl.Frame(kdl.Rotation.RotX(0), kdl.Vector(-168, 0, 0))

frms.append(frm1)
frms.append(frm2)
frms.append(frm3)
frms.append(frm4)
frms.append(frm5)
frms.append(frm6)

rbt = kdl.Chain()  # 建立机器人对象

link = []
for i in range(6):
    link.append(kdl.Segment(jnts[i], frms[i]))
    rbt.addSegment(link[i])

fk = kdl.ChainFkSolverPos_recursive(rbt)

p = kdl.Frame()
q = kdl.JntArray(6)
q[0] = 0
q[1] = 0
q[2] = 0
q[3] = 0
q[4] = 0
q[5] = 0

fk.JntToCart(q, p)

print(p)

##建立正运动学