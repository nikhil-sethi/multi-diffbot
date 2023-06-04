import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class RobotPlanner:
    def __init__(self) -> None:
        rospy.init_node('mirte_client_py')
        self.r_safe = 0.3
        self.goal_pub = rospy.Publisher("mirte/move_base_simple/goal", PoseStamped, queue_size=10)
        
        self.farmer_pose_sub = rospy.Subscriber("farmer/gazebo/odom_gt", Odometry, self.farmer_pose_update, queue_size=10)
        self.farmer_pose = Pose2D()
        self.robot_pose_sub = rospy.Subscriber("mirte/gazebo/odom_gt", Odometry, self.robot_pose_update, queue_size=10)
        self.robot_pose = Pose2D()

        timer = rospy.Timer(rospy.Duration.from_sec(0.2), self.run)

    @staticmethod
    def pose2d_from_odom(odom_msg:Odometry):
        full_pose:Pose = odom_msg.pose.pose
        # angles = euler_from_quaternion(full_pose.orientation)
        return Pose2D(full_pose.position.x, full_pose.position.y, 2*math.acos(full_pose.orientation.w))

    def farmer_pose_update(self, msg:Odometry):
        self.farmer_pose = self.pose2d_from_odom(msg)

    def bull_pose_update(self, msg:Odometry):
        self.bull_pose = self.pose2d_from_odom(msg)

    def robot_pose_update(self, msg:Odometry):
        self.robot_pose = self.pose2d_from_odom(msg)

    def run(self, event=None):
        dx = self.farmer_pose.x - self.robot_pose.x
        dy = self.farmer_pose.y - self.robot_pose.y

        mag = math.sqrt(dx**2 + dy**2)
        
        # unit vectors
        # dx_c = dx/mag
        # dy_c = dy/mag 

        # theta_opt = math.tan(dy/dx)
        theta_opt = self.farmer_pose.theta
        x_opt = self.farmer_pose.x - self.r_safe*math.cos(theta_opt)
        y_opt =  self.farmer_pose.y - self.r_safe*math.sin(theta_opt)
        
        pose_opt = PoseStamped()
        pose_opt.header.frame_id = "mirte_tf/map"
        pose_opt.header.stamp = rospy.Time.now()
        pose_opt.pose.position.x = x_opt
        pose_opt.pose.position.y = y_opt
        pose_opt.pose.orientation.w = math.cos(theta_opt/2)
        pose_opt.pose.orientation.z = math.sin(theta_opt/2)

        self.goal_pub.publish(pose_opt)

if __name__=="__main__":
    robot_planner = RobotPlanner()
    rospy.spin()