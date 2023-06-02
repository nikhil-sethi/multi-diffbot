#!/usr/bin/env python3

"""
Simple sub/pub to forward messages from the physical to the simulated cmd_vel topic
"""

import rospy
from geometry_msgs.msg import Twist

def message_callback(msg):
    """ Callback function to process the received message """
    # Publish the received message to another topic
    publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('vel_fwd')  # Initialize the ROS node
    rospy.loginfo("vel forward Node Started")
    
    # Create a publisher to publish messages to the other topic
    publisher = rospy.Publisher('/mirte/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    
    # Create a subscriber to listen to the input topic
    rospy.Subscriber('/mobile_base_controller/cmd_vel', Twist, message_callback)
    
    rospy.spin()  # Keep the node running until it's stopped