#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('hello_transmitter', anonymous=True)
    hello_pub = rospy.Publisher('hello_topic', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        hello_str = "Hello, world!"
        rospy.loginfo(hello_str)
        hello_pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
