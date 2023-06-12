#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def callback(data):
    rospy.logwarn("WARNING: Aggressive cow detected.")

def listener():
    rospy.init_node('alert_farmer', anonymous=True)
    rospy.Subscriber('/alert', Bool, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()