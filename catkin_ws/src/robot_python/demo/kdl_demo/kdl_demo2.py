import PyKDL
from tf_conversions import posemath
# frame
f = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0),
                PyKDL.Vector(3,2,4))
print "f:%s" % f

# get the origin (a Vector) of the frame
origin = f.p
print "origin:%s" % origin

# get the x component of the origin
x = origin.x()
print "x:%s" % x
x = origin[0]

# get the rotation of the frame
rot = f.M
print "rot:%s" % rot

# get ZYX Euler angles from the rotation
[Rz, Ry, Rx] = rot.GetEulerZYX()

# get the RPY (fixed axis) from the rotation
[R, P, Y] = rot.GetRPY()



# you have a Pose message
pose = posemath.Pose()
print "pose:%s" % pose

# convert the pose into a kdl frame
f1 = posemath.fromMsg(pose)
print "f1:%s" % f1
# create another kdl frame
f2 = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0),
                 PyKDL.Vector(3,2,4))

# Combine the two frames
f = f1 * f2

pose = posemath.toMsg(posemath.fromMsg(pose) * PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0), PyKDL.Vector(3,2,4)))
print "pose:%s" % pose

# and convert the result back to a pose message
pose = posemath.toMsg(f)