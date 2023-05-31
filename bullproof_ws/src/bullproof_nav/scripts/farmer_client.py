#!/usr/bin/env python

# import rospy
# import random
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Pose2D, Point
# from actionlib import SimpleActionClient, SimpleActionServer
# from bullproof_nav.msg import NavPose2DAction, NavPose2DGoal

# class GoalGenerator:
#     """ ROS action client """
#     def __init__(self) -> None:
#         self.client = SimpleActionClient("do_farmer")
#         self.client.wait_for_server()

#         # define goals
#         self.wps = [Pose2D(2.6, 1.2, 0),
#            Pose2D(2.6, 2.6, 1.57),
#            Pose2D(0.4, 2.6, 3.14),
#            Pose2D(0.4, 1.2, -1.57)]
        
#         self.wp_counter = 0

#     def send_goal(self):
#         if self.reached():
#             self.wp_counter += 1
#             goal = self.wps[self.wp_counter%len(self.wps)]
            
#             self.client.send_goal(goal)

#     def reached(self):
#         self.client.get_result()

#     def run(self):
#         self.send_goal()
#         self.wait_for_result(rospy.Duration.from_sec(5.0))
        

# if __name__=="__main__":
    # rospy.init_node('do_farmer_nav_client')
    # client = SimpleActionClient('do_farmer_nav', NavPose2DAction)
    # client.wait_for_server()
    # goal = NavPose2DGoal()
    # goal.pose.x = 2.6
    # goal.pose.y = 1.23
    # goal.pose.theta = 0
    # # Fill in the goal here
    # client.send_goal(goal)
    # print("Sent goal to server")
    # client.wait_for_result(rospy.Duration.from_sec(15.0))


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "farmer/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")