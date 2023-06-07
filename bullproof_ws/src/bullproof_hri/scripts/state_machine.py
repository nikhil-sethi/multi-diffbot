#!/usr/bin/env python

import os
import threading
import smach
import smach_ros
from pynput import keyboard
import rospy
from std_msgs.msg import Int32
# from bullproof_interfaces.msg import RobotState, BullState, FarmerState
from enum import IntEnum

class RobotRole(IntEnum):
    CLEAN = 0
    FOLLOW = 1
    PROTECT = 2
    STAY = 3

class RoleManager:
    def __init__(self) -> None:
        self.pub = rospy.Publisher('mirte/state', Int32, queue_size=10)

        timer = rospy.Timer(rospy.Duration.from_sec(1), self.role_publisher)
        self.mirte_state = 0 # clean by default
        
    def role_publisher(self, event=None):
        self.pub.publish(self.mirte_state)


class Clean_Stable(smach.State):
    """ Robot cleans stable, no farmer detected (F2 in activity diagram) """
    def __init__(self, role_manager):
        smach.State.__init__(self, outcomes=['start_following', 'quit'])
        self.role_manager = role_manager

    def execute(self, userdata):
        # Start keyboard listener thread
        rospy.loginfo("State Clean_Stable triggered. (e to transition to Follow_farmer, q to quit)" )
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        self.listener.join()  # Wait for keyboard listener thread to stop
        return self.next_state
        
    def on_key_press(self, key):
        try:
            if key.char == 'e':
                self.listener.stop()  # Stops listener
                self.next_state = 'start_following'
                self.role_manager.mirte_state = RobotRole.FOLLOW
            if key.char =='q':
                self.listener.stop()
                self.next_state = 'quit'
        except AttributeError: # if it is not a character, such as 'Shift'
            pass

class Follow_Farmer(smach.State):
    """ Robot follows farmer, farmer has been detected but no aggressive cow (F3 in activity diagram) """
    def __init__(self, role_manager):
        smach.State.__init__(self, outcomes=['start_cleaning', 'start_protecting', 'quit'])
        self.role_manager = role_manager

    def execute(self, userdata):
        # Start keyboard listener thread
        rospy.loginfo("State Follow_Farmer triggered. (e to transition to Clean_Stable, r to transition to Protect_Farmer, q to quit)")
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        self.listener.join()  # Wait for keyboard listener thread to stop
        return self.next_state
    
    def on_key_press(self, key):
        try:
            if key.char == 'e':
                self.listener.stop()  # Stops listener
                self.next_state = 'start_cleaning'
                self.role_manager.mirte_state = RobotRole.CLEAN
            if key.char == 'r':
                self.listener.stop()  
                self.next_state = 'start_protecting'
                self.role_manager.mirte_state = RobotRole.PROTECT
            if key.char == 'q':
                self.listener.stop()
                self.next_state = 'quit'
        except AttributeError: # if it is not a character, such as 'Shift'
            pass


class Protect_Farmer(smach.State):
    """ Robot protects farmer, both farmer and aggressive cow are detected (F6 in activity diagram) """
    def __init__(self, role_manager):
        smach.State.__init__(self, outcomes=['start_following', 'quit'])
        self.role_manager = role_manager

    def execute(self, userdata):
        # Start keyboard listener thread
        rospy.loginfo("State Protect_Farmer triggered. (r to transition to Follow_Farmer, q to quit)")
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        self.listener.join()  # Wait for keyboard listener thread to stop

        return self.next_state

    def on_key_press(self, key):
        try:
            if key.char == 'r':
                self.listener.stop()  # Stops listener
                self.next_state = 'start_following'
                self.role_manager.mirte_state = RobotRole.FOLLOW
            if key.char =='q':
                self.listener.stop()
                self.next_state = 'quit'
        except AttributeError:
            pass

def main():
    rospy.init_node('state_machine_node')
    sm = smach.StateMachine(outcomes=['exit'])

    with sm:
        role_manager = RoleManager()
        smach.StateMachine.add('Clean_Stable', Clean_Stable(role_manager), transitions={'start_following': 'Follow_Farmer', 
                                                                            'quit': 'exit'})
        smach.StateMachine.add('Follow_Farmer', Follow_Farmer(role_manager), transitions={'start_cleaning': 'Clean_Stable',
                                                                              'start_protecting': 'Protect_Farmer',
                                                                              'quit': 'exit'})
        smach.StateMachine.add('Protect_Farmer', Protect_Farmer(role_manager), transitions={'start_following': 'Follow_Farmer',
                                                                                'quit': 'exit'})
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('sm_server', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    sm.set_initial_state(['Clean_Stable'])
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()