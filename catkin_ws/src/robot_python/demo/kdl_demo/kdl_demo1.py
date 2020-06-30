#!/usr/bin/python
#-*-coding:utf-8-*-
#创建变量

import PyKDL
import numpy as np

# create a vector
v = PyKDL.Vector(1, 3, 5)
print "v", v

# create a rotation from Roll Pitch, Yaw angles
r1 = PyKDL.Rotation.RPY(1.2, 3.4, 0)
print "r1:\n", r1

#from Roll Pitch, Yaw angles to a rotation
rpy = r1.GetRPY()
print "rpy:", rpy

# create a rotation from XYZ Euler angles
r2 = PyKDL.Rotation.EulerZYX(0, 1, 0)
print"r2:\n", r2

#from XYZ Euler angles to a rotation
EulerZYX = r2.GetEulerZYX()
print "EulerZYX:", EulerZYX

print "Quaternion:", r2.GetQuaternion()

# create a rotation from a rotation matrix
r3 = PyKDL.Rotation(1, 0, 0, 2, 1, 0, 3, 0, 1)
print "r3:\n", r3

#create a rotation from a Quaternion
r4 = PyKDL.Rotation.Quaternion(0.0, 0.47942553860420295, 0.0, 0.8775825618903728)
print "r4:\n", r4

#from a rotation to a Quaternion
Quaternion = r4.GetEulerZYX()
print "Quaternion:", Quaternion

#create a RotAngle from a rotation
[rot_angle, rot_vector] = r4.GetRotAngle()
print "rot_angle:", rot_angle
print "rot_vector:", rot_vector

r5 = PyKDL.Rotation.Rot(rot_vector, rot_angle)
print "r5:\n", r5


# create a frame from a vector and a rotation
f1 = PyKDL.Frame(r1, v)
print "f:\n", f1

# create a frame from a STD DH
theta = 1.57
d = 0.53
a = 0.35
alpha = 3.14

f2 = PyKDL.Frame()
f2 = f2.DH(a, alpha, d, theta)
print "f2:\n", f2

# create a frame from a Craig1989 DH
theta = 1.57
d = 0.53
a = 0.35
alpha = 3.14

f3 = PyKDL.Frame()
f3 = f3.DH_Craig1989(a, alpha, d, theta)
print "f3:\n", f3

#读取坐标原点
p = f3.p
print "P:", p
#读取旋转矩阵
rot = f3.M
print "rot:\n", rot

#求取坐标的逆
frame_inv = f3.Inverse()
print "frame_inv:\n", frame_inv

#create a wrench
wr = PyKDL.Wrench()
wr.force = PyKDL.Vector(0, 1, 2)
wr.torque = PyKDL.Vector(3, 4, 5)
print "wr:\n", wr

#read a wrench
f = wr.force
print "f:", f
M = wr.torque
print "M:", M

#create a Twist
Tw = PyKDL.Twist()
Tw.vel = PyKDL.Vector(0, 1, 2)
Tw.rot = PyKDL.Vector(3, 4, 5)
print "Tw:\n", Tw

#read a Twist
v = Tw.vel
print "v:", v
w = Tw.rot
print "w:", w

#create a jacobian
jac = PyKDL.Jacobian(7)
print "jac", jac.columns()