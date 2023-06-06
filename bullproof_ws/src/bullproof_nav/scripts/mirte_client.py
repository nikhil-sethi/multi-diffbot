import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion
import math
import enum

def euler_from_quaternion(q:Quaternion):
    
    angles = [0,0,0]
    # // roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    angles[0] = math.atan2(sinr_cosp, cosr_cosp)

    # // pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
    cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
    angles[1] = 2 * math.atan2(sinp, cosp) - math.pi / 2

    # // yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    angles[2] = math.atan2(siny_cosp, cosy_cosp)

    return angles


class RobotPlanner:
    def __init__(self) -> None:
        rospy.init_node('mirte_client_py')
        rospy.loginfo("Starting mirte_client_py" )
        self.r_safe = 0.3
        self.goal_pub = rospy.Publisher("mirte/move_base_simple/goal", PoseStamped, queue_size=10)
        
        self.farmer_pose_sub = rospy.Subscriber("farmer/gazebo/odom_gt", Odometry, self.farmer_pose_update, queue_size=10)
        self.farmer_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber("mirte/gazebo/odom_gt", Odometry, self.robot_pose_update, queue_size=10)
        self.robot_pose = Pose()
        
        self.mirte_role_sub = rospy.Subscriber("mirte/robot_role", Int32, self.robot_role_update, queue_size=10)
        self.mirte_role = None

        rospy.sleep(1)
        timer = rospy.Timer(rospy.Duration.from_sec(0.2), self.run)


    @staticmethod
    def pose2d_from_odom(odom_msg:Odometry):
        full_pose:Pose = odom_msg.pose.pose
        # angles = euler_from_quaternion(full_pose.orientation)
        return Pose2D(full_pose.position.x, full_pose.position.y, 2*math.acos(full_pose.orientation.w))

    def farmer_pose_update(self, msg:Odometry):
        self.farmer_pose = msg.pose.pose

    def bull_pose_update(self, msg:Odometry):
        self.bull_pose = self.pose2d_from_odom(msg)

    def robot_pose_update(self, msg:Odometry):
        self.robot_pose = msg.pose.pose

    def robot_role_update(self, msg):
        self.robot_role = msg.data

    def run(self, event=None):
        if self.robot_role == 1: # Following
            dx = self.farmer_pose.position.x - self.robot_pose.position.x
            dy = self.farmer_pose.position.y - self.robot_pose.position.y

            mag = math.sqrt(dx**2 + dy**2)
            
            # unit vectors
            # dx_c = dx/mag
            # dy_c = dy/mag 

            # theta_opt = math.tan(dy/dx)
            angles = euler_from_quaternion(self.farmer_pose.orientation)
            
            # rospy.loginfo(theta_opt)
            x_opt = self.farmer_pose.position.x - self.r_safe*math.cos(angles[-1])
            y_opt =  self.farmer_pose.position.y - self.r_safe*math.sin(angles[-1])
            
            pose_opt = PoseStamped()
            pose_opt.header.frame_id = "mirte_tf/map"
            pose_opt.header.stamp = rospy.Time.now()
            pose_opt.pose.position.x = x_opt
            pose_opt.pose.position.y = y_opt
            pose_opt.pose.orientation = self.farmer_pose.orientation
            # pose_opt.pose.orientation.z = abs(math.sin(theta_opt/2))

            self.goal_pub.publish(pose_opt)

if __name__=="__main__":
    robot_planner = RobotPlanner()
    rospy.spin()