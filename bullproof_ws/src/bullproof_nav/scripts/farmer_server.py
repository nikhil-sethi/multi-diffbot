# from actionlib import SimpleActionServer
# from bullproof_nav.msg import NavPose2DAction
# from geometry_msgs.msg import Twist, Pose2D, Pose
# from nav_msgs.msg import Odometry
# import math

# import rospy


# class GoalExecuter:
#     """ ROS action server"""
#     def __init__(self) -> None:
#         self.server = SimpleActionServer("do_farmer_nav", NavPose2DAction, self.execute, auto_start=False)
        
#         self.vel_pub = rospy.Publisher("/farmer/mobile_base_controller/cmd_vel", Twist, queue_size=10)
#         self.pose = Pose2D()
#         self.pose_sub = rospy.Subscriber("/farmer/gazebo/odom_gt", Odometry, self.pose_callback)
#         self.server.start()

#     def pose_callback(self, msg:Odometry):
#         full_pose:Pose = msg.pose.pose

#         self.pose.x = full_pose.position.x
#         self.pose.y = full_pose.position.y
#         theta = 2*math.acos(full_pose.orientation.w)
#         self.pose.theta = theta

#     def target_vel(self, target_pose):
#         vel = Twist()
#         v_rel = target_pose - self.pose 
#         vel.linear.x = 0.1
#         return vel

#     def execute(self, goal):
#         rospy.loginfo(goal)
#         while True:
#             if self.pos_reached(goal.pose):
#                 vel = self.target_vel(goal.pose)
#                 self.vel_pub.publish(vel)
#             if self.pos_reached(goal.pose):
#                 break
#         self.server.set_succeeded()
        
#     def pos_reached(self, target_pose:Pose2D):
#         tol = 0.1
#         if abs(self.pose.x-target_pose.x)<tol and abs(self.pose.y-target_pose.y)<tol:
#             return True
#         return False
        
#     def theta_reached(self, target_pose:Pose2D):
#         tol = 0.1
#         if abs(self.pose.theta - target_pose.theta)<tol:
#             return True
#         return False
    
# if __name__=="__main__":
#     rospy.init_node("do_farmer_nav_server")
#     print("Starting server")
#     server = GoalExecuter()
#     rospy.spin()


