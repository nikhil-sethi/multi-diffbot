#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import math

class BullPlanner:
    def __init__(self) -> None:
        rospy.init_node('bull_client_py')
        self.wps = [Pose2D(1.2, 1.2, 0),
                    Pose2D(0.3, 0.3, 0),
                    Pose2D(0.5, 0.8, 0),
                    Pose2D(0.8, 1.0, 0),
                    Pose2D(0.7, 0.3, 0)]
        self.wp_counter = 0

        self.goal_pub = rospy.Publisher("bull/move_base_simple/goal", PoseStamped, queue_size=10)
        
        # self.farmer_pose_sub = rospy.Subscriber("farmer/gazebo/odom_gt", Odometry, self.farmer_pose_update, queue_size=10)
        # self.farmer_pose = Pose()
        self.goal_status_sub = rospy.Subscriber("bull/move_base/status", GoalStatusArray, self.goal_status_callback, queue_size=10)

        # self.robot_pose_sub = rospy.Subscriber("mirte/gazebo/odom_gt", Odometry, self.robot_pose_update, queue_size=10)
        # self.robot_pose = Pose()
        rospy.sleep(2)
        self.goal_publish(GoalStatus.SUCCEEDED)# to get things started

    def goal_publish(self, goal_status):
        if goal_status == GoalStatus.SUCCEEDED:
            wp = self.wps[self.wp_counter % len(self.wps)]
            target_pose = PoseStamped()
            target_pose.header.frame_id = "bull_tf/map"
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = wp.x
            target_pose.pose.position.y = wp.y

            target_pose.pose.orientation.w = math.cos(wp.theta/2)
            target_pose.pose.orientation.z = math.sin(wp.theta/2)
            
            self.goal_pub.publish(target_pose)
            self.wp_counter += 1

    def goal_status_callback(self, msg:GoalStatusArray):
        if msg.status_list:
            goal_status = msg.status_list[-1].status
            self.goal_publish(goal_status)

if __name__ == '__main__':

    farmer_planner = BullPlanner()

    rospy.spin()
