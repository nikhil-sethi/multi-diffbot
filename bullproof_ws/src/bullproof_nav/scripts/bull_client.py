#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose2D
from actionlib_msgs.msg import GoalStatusArray
import math

i=0
def movebase_client():

    client = actionlib.SimpleActionClient('/bull/move_base_simple',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/bull/base_link"
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

def goal_publish(status):
    global i
    if status==3:
        wp = goals[i%len(goals)]
        target_pose = PoseStamped()
        target_pose.header.frame_id = "bull_tf/map"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = wp.x
        target_pose.pose.position.y = wp.y

        target_pose.pose.orientation.w = math.cos(wp.theta/2)
        target_pose.pose.orientation.z = math.sin(wp.theta/2)
        
        goal_pub.publish(target_pose)
        i += 1 # for next time

def goal_status_callback(msg:GoalStatusArray):
    if msg.status_list:
        status = msg.status_list[-1].status
        goal_publish(status)


if __name__ == '__main__':
    # try:
    rospy.init_node('bull_client_py')
    #     result = movebase_client()
    #     if result:
    #         rospy.loginfo("Goal execution dgoal_pub = rospy.Publisher("bull/move_base_simple/goal", PoseStamped, queue_size=10)
    goals = [Pose2D(2.0, 1.0, 0),
            Pose2D(1.0, 2.0, 0)]

    goal_pub = rospy.Publisher("bull/move_base_simple/goal", PoseStamped, queue_size=10)
    goal_status_sub = rospy.Subscriber("bull/move_base/status", GoalStatusArray, goal_status_callback, queue_size=10)
    rospy.sleep(2)

    # just to get things started up and set and active status=1
    goal_publish(3)
    
    rospy.spin()
