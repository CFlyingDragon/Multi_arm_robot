import rospy
from testcan.msg import Frame, IpPos

ip_pos = IpPos()
def callback(canMsg):
        ip_pos.id=canMsg.id-0x400
        data=canMsg.data

def talker():
    '''testpub Publisher'''
    rospy.init_node('testpub', anonymous=True)
    pub = rospy.Publisher('can_recieve_position', IpPos, queue_size=10)
    rospy.Subscriber('/canopenexample/can_recieve',Frame, callback)

    while not rospy.is_shutdown():

        pub.publish(sendpos(4,angle*big))
    # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
