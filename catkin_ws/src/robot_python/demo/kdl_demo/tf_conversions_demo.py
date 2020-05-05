#! /usr/bin/env python
import rospy
import PyKDL
from tf_conversions import posemath
from geometry_msgs.msg import Pose

# you have a Pose message
pose = Pose()

pose.position.x = 1
pose.position.y = 1
pose.position.z = 1
pose.orientation.x = pose.orientation.y = pose.orientation.z = 0
pose.orientation.w = 1

# convert the pose into a kdl frame
f1 = posemath.fromMsg(pose)

# create another kdl frame
f2 = PyKDL.Frame(PyKDL.Rotation.RPY(0,1,0),
                 PyKDL.Vector(3,2,4))

# Combine the two frames
f = f1 * f2
print f

[x, y, z, w] = f.M.GetQuaternion()
print x,y,z,w

# and convert the result back to a pose message
pose = posemath.toMsg(f)

pub = rospy.Publisher('pose', Pose, queue_size=1)
rospy.init_node('test', anonymous=True)
rate = rospy.Rate(1) # 1hz

while not rospy.is_shutdown():
    pub.publish(pose)
    rate.sleep()