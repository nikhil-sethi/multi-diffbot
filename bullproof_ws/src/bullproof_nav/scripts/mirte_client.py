import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
import math

farmer_pose = Pose2D()
bull_pose = Pose2D()
pose = Pose2D()
r_safe = 0.4

def farmer_pose_callback(msg:Odometry):
    global farmer_pose

    full_pose:Pose = msg.pose.pose

    farmer_pose.x = full_pose.position.x
    farmer_pose.y = full_pose.position.y
    theta = 2*math.acos(full_pose.orientation.w)
    farmer_pose.theta = theta


def bull_pose_callback(msg:Odometry):
    global bull_pose

    full_pose:Pose = msg.pose.pose

    bull_pose.x = full_pose.position.x
    bull_pose.y = full_pose.position.y
    theta = 2*math.acos(full_pose.orientation.w)
    bull_pose.theta = theta


def pose_callback(msg:Odometry):
    global pose
    full_pose:Pose = msg.pose.pose

    pose.x = full_pose.position.x
    pose.y = full_pose.position.y
    theta = 2 * math.acos(full_pose.orientation.w)
    pose.theta = theta
    

def nav_func(event=None):
    global goal_pub, farmer_pose, bull_pose, pose
    dx = farmer_pose.x - pose.x
    dy = farmer_pose.y - pose.y

    mag = math.sqrt(dx**2 + dy**2)
    
    # unit vectors
    dx_c = dx/mag
    dy_c = dy/mag 

    # theta_opt = math.tan(dy/dx)
    theta_opt = farmer_pose.theta
    x_opt = farmer_pose.x - r_safe*dx_c
    y_opt =  farmer_pose.y - r_safe*dy_c
    
    pose_opt = PoseStamped()
    pose_opt.header.frame_id = "mirte_tf/map"
    pose_opt.header.stamp = rospy.Time.now()
    pose_opt.pose.position.x = x_opt
    pose_opt.pose.position.y = y_opt
    pose_opt.pose.orientation.w = math.cos(theta_opt/2)
    pose_opt.pose.orientation.z = math.sin(theta_opt/2)
    goal_pub.publish(pose_opt)
    

if __name__=="__main__":
    rospy.init_node('mirte_client_py')
    
    goals = [Pose2D(2.6, 1.2, 0),
            Pose2D(2.6, 2.6, 1.57),
            Pose2D(0.4, 2.6, 3.14),
            Pose2D(0.4, 1.2, -1.57)]
    # i=0
    goal_pub = rospy.Publisher("mirte/move_base_simple/goal", PoseStamped, queue_size=10)
    farmer_pose_sub = rospy.Subscriber("farmer/gazebo/odom_gt", Odometry, farmer_pose_callback, queue_size=10)
    pose_sub = rospy.Subscriber("mirte/gazebo/odom_gt", Odometry, pose_callback, queue_size=10)
    rospy.sleep(2)

    timer = rospy.Timer(rospy.Duration.from_sec(0.2), nav_func)
    # just to get things started up and set and active status=1
    # goal_publish(3)
    
    rospy.spin()