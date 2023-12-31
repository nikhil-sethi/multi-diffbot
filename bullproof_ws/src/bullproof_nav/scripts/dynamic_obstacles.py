import rospy
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
import math

class DynamicObstaclesPublisher:
    """Publishes an array of dynamic obstacle poses for the teb local planner"""

    def __init__(self) -> None:
        rospy.init_node("dynamic_obstacle_publisher")

        # pose topics
        self.farmer_odom = Odometry()
        self.bull_odom = Odometry()
        self.robot_odom = Odometry()

        self.farmer_odom_sub = rospy.Subscriber("/farmer/odom", Odometry, self.farmer_odom_callback, queue_size=10)
        self.bull_odom_sub = rospy.Subscriber("/bull/odom", Odometry, self.bull_odom_callback, queue_size=10)

        self.robot_obs_pub = rospy.Publisher("/mirte/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=1)
        obs_timer = rospy.Timer(rospy.Duration.from_sec(0.05), self.publish_obstacles)

    def farmer_odom_callback(self, msg:Odometry):
        self.farmer_odom = msg

    def bull_odom_callback(self, msg:Odometry):
        self.bull_odom = msg

    def robot_odom_callback(self, msg:Odometry):
        self.robot_odom = msg
        
    def obs_from_odom(self, id, name):
        obs_msg = ObstacleMsg()
        odom_msg:Odometry = self.__dict__[name + "_odom"]
        obs_msg.id = id
        obs_msg.polygon.points = [odom_msg.pose.pose.position]
        obs_msg.orientation = odom_msg.pose.pose.orientation
        obs_msg.velocities.twist = odom_msg.twist.twist

        return obs_msg
    
    def publish_obstacles(self, event=None):
        self.obs_array_msg = ObstacleArrayMsg()
        self.obs_array_msg.header.stamp = rospy.Time.now()
        self.obs_array_msg.header.frame_id = "mirte_tf/map"

        # Farmer obstacle
        farmer_obs = self.obs_from_odom(1, "farmer")
        bull_obs = self.obs_from_odom(2, "bull")

        self.obs_array_msg.obstacles = [farmer_obs, bull_obs]

        self.robot_obs_pub.publish(self.obs_array_msg)


if __name__=="__main__":
    dyn_obs_pub = DynamicObstaclesPublisher()
    rospy.spin()
