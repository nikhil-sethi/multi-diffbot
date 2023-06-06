import math
import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID

def euler_from_quaternion(q:Quaternion):
    """ Conversion script to get Euler Angles from Quaternions"""
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


def pose_to_posestamped(pose_msg):
    # Create a new PoseStamped message
    pose_stamped_msg = PoseStamped()

    # Copy details
    pose_stamped_msg.pose = pose_msg.pose.pose

    return pose_stamped_msg


class RobotPlanner:
    def __init__(self) -> None:
        rospy.init_node('mirte_client_py')
        rospy.loginfo("Starting mirte_client_py" )
        self.r_safe = 0.3
        # Goal publisher for mirte
        self.goal_pub = rospy.Publisher("mirte/move_base_simple/goal", PoseStamped, queue_size=10)
        
        # Pose subscribers for farmer and robot
        self.farmer_pose_sub = rospy.Subscriber("farmer/gazebo/odom_gt", Odometry, self.farmer_pose_update, queue_size=10)
        self.farmer_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber("mirte/gazebo/odom_gt", Odometry, self.robot_pose_update, queue_size=10)
        self.robot_pose = Pose()
        
        # Robot role subscriber for state machine
        self.robot_role_sub = rospy.Subscriber("mirte/robot_role", Int32, self.robot_role_update, queue_size=10)
        self.robot_role = None
        self.prev_role = None

        # Waypoints for Mirte cleaning
        self.wps = [Pose2D(2.6, 1.2, 0),
                Pose2D(2.6, 2.6, 1.57),
                Pose2D(0.4, 2.6, 3.14),
                Pose2D(0.4, 1.2, 4.71)]
        self.wp_counter = 0

        # Subscriber for Mirte cleaning waypoint status
        self.waypoint_status_sub = rospy.Subscriber("mirte/move_base/status", GoalStatusArray, self.waypoint_status_callback, queue_size=10)
        rospy.sleep(2)
        self.waypoint_publish(GoalStatus.SUCCEEDED)# to get things started
        self.waypoint_status = GoalStatus.ACTIVE # Then init goal status to active

        # Start repeating with timer
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

    def waypoint_publish(self, waypoint_status):
        if waypoint_status == GoalStatus.SUCCEEDED:
            wp = self.wps[self.wp_counter % len(self.wps)]
            target_pose = PoseStamped()
            target_pose.header.frame_id = "mirte_tf/map"
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = wp.x
            target_pose.pose.position.y = wp.y

            target_pose.pose.orientation.w = math.cos(wp.theta/2)
            target_pose.pose.orientation.z = math.sin(wp.theta/2)
            
            self.goal_pub.publish(target_pose)
            self.wp_counter += 1

    def waypoint_status_callback(self, msg:GoalStatusArray):
        if msg.status_list:
            self.waypoint_status = msg.status_list[-1].status

    def cancel_current_goal(self):
        cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
        goal_id = GoalID()
        goal_id.id = ""
        cancel_pub.publish(goal_id)

    def run(self, event=None):
        # Reset navgoal if role change occured
        if self.prev_role != self.robot_role:
            self.cancel_current_goal()
            self.prev_role = self.robot_role

        if self.robot_role == 0: # Clean stable
            self.waypoint_publish(self.waypoint_status)

        elif self.robot_role == 1: # Following
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