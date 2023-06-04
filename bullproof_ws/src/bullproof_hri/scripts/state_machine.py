#!/usr/bin/env python

import smach
import smach_ros
from pynput import keyboard
import rospy


class Clean_Stable(smach.State):
    """ Robot cleans stable, no farmer detected (F2 in activity diagram) """
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_following', 'quit'])

    def execute(self, userdata):
        # Put code that has to be executed during the state here. 
        rospy.loginfo("State Clean_Stable triggered")
        while True:
            transition = input("Give key input (e to transition to Follow_Farmer, q to quit):")
            if transition == 'e':
                return 'start_following'  # Transition to Follow_Farmer
            if transition == 'q':
                return 'quit'
            else: 
                print("Invalid response")


class Follow_Farmer(smach.State):
    """ Robot follows farmer, farmer has been detected but no aggressive cow (F3 in activity diagram) """
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_cleaning', 'start_protecting', 'quit'])

    def execute(self, userdata):
        # Put code that has to be executed during the state here. 
        rospy.loginfo("State Follow_Farmer triggered")
        while True:
            transition = input("Give key input (e to transition to Clean_Stable, r to transition to Protect_Farmer, q to quit):")
            if transition == 'e':
                return 'start_cleaning'  # Transition to FolloW_Farmer
            elif transition == 'r':
                return 'start_protecting' # Transition to Protect_Farmer
            elif transition == 'q': # Quit
                return 'quit'
            else: 
                print("Invalid response")


class Protect_Farmer(smach.State):
    """ Robot protects farmer, both farmer and aggressive cow are detected (F6 in activity diagram) """
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_following', 'quit'])

    def execute(self, userdata):
        # Put code that has to be executed during the state here. 
        rospy.loginfo("State Protect_Farmer triggered")
        while True:
            transition = input("Give key input (e to transition to Follow_Farmer, q to quit):")
            if transition == 'e':
                return 'start_following'  # Transition to Follow_Farmer
            else: 
                print("Wrong response")


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
    sm.set_initial_state(['Follow_Farmer'])
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()