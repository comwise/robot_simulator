#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def callback_data(p):
    rospy.loginfo("data: %s", p.data)

if __name__ == "__main__":

    rospy.init_node("test_sub")
    sub = rospy.Subscriber("test_data", String, callback_data)
    rospy.spin()