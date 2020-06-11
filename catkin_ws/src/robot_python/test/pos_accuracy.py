#!/usr/bin/env python
'''testpub ROS Node'''
# license removed for brevity
import rospy
from testcan.msg import Frame, IpPos
import time

big = 2498560.0/360.0
degreetoradius = 180.0/3.1415926
radiustodegree = 3.1415926/180.0

ip_pos = IpPos()
def sendpos(id, pos):
        ip_pos.pos = pos
        ip_pos.id = id
        return ip_pos

def talker():
    '''testpub Publisher'''
    pub = rospy.Publisher('ip_pos', IpPos, queue_size=10)
    rospy.init_node('testpub', anonymous=True)
    # ip_pos = IpPos()
    pub.publish(sendpos(2,0))
    time.sleep(3)
    # rate = rospy.Rate(1) # 10hz
    motor_vel = 10
    motor_vel_slow = 0.1
    step = 4*int(832*motor_vel/160)
    angle = 45 
    stepangle = -45
    pos = 0
    count = 0
    print("currently velocity of motor is %f\n"%(step*160.0/416))
    while not rospy.is_shutdown():
        # count = count + 1
        # # if count == 250:
        # #     motor_vel = motor_vel + 45
        # #     step = int(832*motor_vel/160)
        # #     print("currently velocity of motor is %d\n"%motor_vel)
        # #     count = 0 
        # if count == 2500:
        motor_vel = motor_vel + 0.1
        # step = step + int(832*motor_vel/160)
        print("currently velocity of motor is %f\n"%(step*160.0/416))
                    # count = 0 
        pub.publish(sendpos(4,angle*big))
        # angle*big*degreetoradius
        # rate.sleep()
        print(angle*big)
        # angle = angle + stepangle
        angle = -angle
        time.sleep(10)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
