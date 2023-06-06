#!/usr/bin/env python

import os
import threading
import smach
import smach_ros
from pynput import keyboard
import rospy
from std_msgs.msg import Int32

class Clean_Stable(smach.State):
    """ Robot cleans stable, no farmer detected (F2 in activity diagram) """
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_following', 'quit'])
        self.pub = rospy.Publisher('robot_role', Int32, queue_size=10)
        self.publish_thread = None
        self.run = False


    def execute(self, userdata):
        # Clear console
        os.system('clear')

        # Start publishing thread
        self.run = True 
        self.publish_thread = threading.Thread(target=self.publish_robot_role)
        self.publish_thread.daemon = True
        self.publish_thread.start()

        # Start keyboard listener thread
        rospy.loginfo("State Clean_Stable triggered. (e to transition to Follow_farmer, q to quit)" )
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        
        self.listener.join()  # Wait for keyboard listener thread to stop

        self.run = False  # Turn off publishing thread
        self.publish_thread.join()  # Wait for the publishing thread to stop

        return self.next_state
        
    def on_key_press(self, key):
        try:
            if key.char == 'e':
                self.listener.stop()  # Stops listener
                self.next_state = 'start_following'
                self.run, self.publish_thread.daemon = False
            if key.char =='q':
                self.listener.stop()
                self.next_state = 'quit'
                self.run, self.publish_thread.daemon = False
        except AttributeError:
            pass
    
    def publish_robot_role(self):
        while self.run and not rospy.is_shutdown():
            role_value = 0  # Modify this value if needed
            self.pub.publish(role_value)
            rospy.Rate(1).sleep() # Publish rate of 1 Hz


class Follow_Farmer(smach.State):
    """ Robot follows farmer, farmer has been detected but no aggressive cow (F3 in activity diagram) """
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_cleaning', 'start_protecting', 'quit'])
        self.pub = rospy.Publisher('robot_role', Int32, queue_size=10)
        self.publish_thread = None
        self.run = False

    def execute(self, userdata):
        # Clear console
        os.system('clear')

        # Start publishing thread
        self.run = True 
        self.publish_thread = threading.Thread(target=self.publish_robot_role)
        self.publish_thread.daemon = True
        self.publish_thread.start()

        # Start keyboard listener thread
        rospy.loginfo("State Follow_Farmer triggered. (e to transition to Clean_Stable, r to transition to Protect_Farmer, q to quit)")
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        
        self.listener.join()  # Wait for keyboard listener thread to stop

        self.run = False  # Turn off publishing thread
        self.publish_thread.join()  # Wait for the publishing thread to stop

        return self.next_state
    
    def on_key_press(self, key):
        try:
            if key.char == 'e':
                self.listener.stop()  # Stops listener
                self.next_state = 'start_cleaning'
            if key.char == 'r':
                self.listener.stop()  
                self.next_state = 'start_protecting'
            if key.char == 'q':
                self.listener.stop()
                self.next_state = 'quit'
        except AttributeError:
            pass

    def publish_robot_role(self):
        while self.run and not rospy.is_shutdown():
            role_value = 1  # Modify this value if needed
            self.pub.publish(role_value)
            rospy.Rate(1).sleep() # Publish rate of 1 Hz

class Protect_Farmer(smach.State):
    """ Robot protects farmer, both farmer and aggressive cow are detected (F6 in activity diagram) """
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_following', 'quit'])
        self.pub = rospy.Publisher('robot_role', Int32, queue_size=10)
        self.publish_thread = None
        self.run = False

    def execute(self, userdata):
        # Clear console
        os.system('clear')

        # Start publishing thread
        self.run = True 
        self.publish_thread = threading.Thread(target=self.publish_robot_role)
        self.publish_thread.daemon = True
        self.publish_thread.start()

        # Start keyboard listener thread
        rospy.loginfo("State Protect_Farmer triggered. (r to transition to Follow_Farmer, q to quit)")
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        
        self.listener.join()  # Wait for keyboard listener thread to stop

        self.run = False  # Turn off publishing thread
        self.publish_thread.join()  # Wait for the publishing thread to stop

        return self.next_state

    def on_key_press(self, key):
        try:
            if key.char == 'r':
                self.listener.stop()  # Stops listener
                self.next_state = 'start_following'
            if key.char =='q':
                self.listener.stop()
                self.next_state = 'quit'
        except AttributeError:
            pass

    def publish_robot_role(self):
        while self.run and not rospy.is_shutdown():
            role_value = 2 # Modify this value if needed
            self.pub.publish(role_value)
            rospy.Rate(1).sleep() # Publish rate of 1 Hz

def main():
    rospy.init_node('state_machine_node')
    sm = smach.StateMachine(outcomes=['exit'])

    with sm:
        smach.StateMachine.add('Clean_Stable', Clean_Stable(), transitions={'start_following': 'Follow_Farmer', 
                                                                            'quit': 'exit'})
        smach.StateMachine.add('Follow_Farmer', Follow_Farmer(), transitions={'start_cleaning': 'Clean_Stable',
                                                                              'start_protecting': 'Protect_Farmer',
                                                                              'quit': 'exit'})
        smach.StateMachine.add('Protect_Farmer', Protect_Farmer(), transitions={'start_following': 'Follow_Farmer',
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