#!/usr/bin/env python3
import rospy
from pynput import keyboard
from std_msgs.msg import Int32

# Global variables to store motor commands
left_motor_cmd = 0
right_motor_cmd = 0

def control_mirte():
    # Create function to execute motor commands on key input (manual control)
    def on_press(key):
        global left_motor_cmd, right_motor_cmd

        inp_strength = 500
        if key == keyboard.Key.up:
            left_motor_cmd = inp_strength
            right_motor_cmd = inp_strength
        elif key == keyboard.Key.down:
            left_motor_cmd = -inp_strength
            right_motor_cmd = -inp_strength
        elif key == keyboard.Key.left:
            left_motor_cmd = -inp_strength
            right_motor_cmd = inp_strength
        elif key == keyboard.Key.right:
            left_motor_cmd = inp_strength
            right_motor_cmd = -inp_strength

        if key == keyboard.Key.esc: # exit out of input listen loop
            return False


    # Stop robot if key is released
    def on_release(key):
        global left_motor_cmd, right_motor_cmd
        left_motor_cmd = 0
        right_motor_cmd = 0
    

    rospy.init_node('keyboard_control')

    # Create publishers for left and right motor commands
    left_motor_pub = rospy.Publisher('/mirte/motor_left_speed', Int32, queue_size=10)
    right_motor_pub = rospy.Publisher('/mirte/motor_right_speed', Int32, queue_size=10)

    rate = rospy.Rate(10)  # Publish rate in Hz
    
    # Create the keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while not rospy.is_shutdown():
        left_motor_pub.publish(left_motor_cmd)
        right_motor_pub.publish(right_motor_cmd)

        rate.sleep()
    return

if __name__ == '__main__':
    try:
        control_mirte()
    except rospy.ROSInterruptException:
        pass
