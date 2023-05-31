#!/usr/bin/env python3
import rospy
from pynput import keyboard
from geometry_msgs.msg import Twist

# Global variables to store motor commands
move_cmd = Twist()
stopInput = False

def control_mirte():
    # Create function to execute motor commands on key input (manual control)
    def on_press(key):
        global move_cmd, stopInput
        inp_strength = 2
        rad_strength = 2
        if not stopInput:
            if key == keyboard.Key.up:
                move_cmd.linear.x += inp_strength
            if key == keyboard.Key.down:
                move_cmd.linear.x -= inp_strength
            if key == keyboard.Key.left:
                move_cmd.angular.z += rad_strength
            if key == keyboard.Key.right:
                move_cmd.angular.z -= rad_strength
            stopInput = True
        if key == keyboard.Key.esc: # exit out of input listen loop
            return False
        

    # Stop robot if key is released
    def on_release(key):
        global move_cmd, stopInput
        stopInput = False
        move_cmd = Twist()
    

    rospy.init_node('keyboard_control')

    # Create publishers for left and right motor commands
    movecmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # Publish rate in Hz
    
    # Create the keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while not rospy.is_shutdown():
        movecmd_pub.publish(move_cmd)
        rate.sleep()
    return

if __name__ == '__main__':
    try:
        control_mirte()
    except rospy.ROSInterruptException:
        pass
