#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

max_forward = 0.42
max_angular = 1.7

def on_press(key):
    try:
        if key == keyboard.Key.up:
            move_forward()
        elif key == keyboard.Key.down:
            move_backward()
        elif key == keyboard.Key.left:
            turn_left()
        elif key == keyboard.Key.right:
            turn_right()
    except AttributeError:
        pass

def on_release(key):
    stop_motion()

def move_forward():
    twist = Twist()
    twist.linear.x = max_forward # Set the linear velocity to move forward
    cmd_vel_pub.publish(twist)

def move_backward():
    twist = Twist()
    twist.linear.x = -max_forward # Set the linear velocity to move backward
    cmd_vel_pub.publish(twist)

def turn_left():
    twist = Twist()
    twist.angular.z = max_angular  # Set the angular velocity to turn left
    cmd_vel_pub.publish(twist)

def turn_right():
    twist = Twist()
    twist.angular.z = -max_angular  # Set the angular velocity to turn right
    cmd_vel_pub.publish(twist)

def stop_motion():
    twist = Twist()  # Set all velocities to 0 to stop the motion
    cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('keyboard_control_node')
    cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    rospy.spin()
