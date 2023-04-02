#! /usr/bin/env python

from os import putenv
import rospy
from std_msgs.msg import String

if __name__ == "__main__":

    rospy.init_node("test_pub")

    pub = rospy.Publisher("test_data", String, queue_size=10)
    
    rate = rospy.Rate(1)
    msg = String()
    count = 1
    while not rospy.is_shutdown():
        count += 1
        msg.data = "Hello, I'm sandi" + str(count)
        print(msg)
        pub.publish(msg)
        rate.sleep()
