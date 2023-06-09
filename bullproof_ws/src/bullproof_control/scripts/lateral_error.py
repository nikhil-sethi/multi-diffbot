import rospy
import numpy as np
import time
import math
import rosbag
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Point
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

local_errors = []
global_errors = []
cur_pose = Point()
local_des_poses = PoseStamped()
global_des_poses = PoseStamped()

def odometry_callback(msg):
    global cur_pose
    # Process the odometry data, calculate lateral error
    cur_pose = msg.pose.pose

def local_trajectory_callback(msg):
    global local_des_poses
    # Process the trajectory data, store the desired trajectory
    local_des_poses = msg.poses[2]

def global_trajectory_callback(msg):
    global global_des_poses
    # Process the trajectory data, store the desired trajectory
    global_des_poses = msg.poses[0]
  
def calculate_lateral_error(current_pose, desired_poses):
    x1, y1 = current_pose.position.x, current_pose.position.y
    orientation_rad = euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])[-1]

    desired_pose = desired_poses

    x2, y2 = desired_pose.pose.position.x, desired_pose.pose.position.y

    # Calculate the vector between the two points
    vector_x = x2 - x1
    vector_y = y2 - y1

    # Calculate the perpendicular vector to the orientation
    perpendicular_x = math.cos(orientation_rad + math.pi / 2)
    perpendicular_y = math.sin(orientation_rad + math.pi / 2)

    # Calculate the dot product of the vector and the perpendicular vector
    dot_product = vector_x * perpendicular_x + vector_y * perpendicular_y

    # Calculate the magnitude of the perpendicular vector
    perpendicular_magnitude = math.sqrt(perpendicular_x ** 2 + perpendicular_y ** 2)

    # Calculate the lateral error
    lateral_error = abs(dot_product / perpendicular_magnitude)
    
    return lateral_error


if __name__ == '__main__':
    rospy.init_node('lateral_trajectory_error_node')
    rospy.Subscriber('/mirte/gazebo/odom_gt', Odometry, odometry_callback)
    rospy.Subscriber('/mirte/move_base/TebLocalPlannerROS/local_plan', Path, local_trajectory_callback)
    rospy.Subscriber('/mirte/move_base/TebLocalPlannerROS/global_plan', Path, global_trajectory_callback)

    local_error_publisher = rospy.Publisher('/mirte/local_trajectory_error', Float32, queue_size=10)
    global_error_publisher = rospy.Publisher('/mirte/global_trajectory_error', Float32, queue_size=10)

    bag = rosbag.Bag('bullproof_ws/src/bullproof_control/bagfiles/lateral_trajectory_error.bag', 'w')  # Create a bag file
    hz = 10
    rate = rospy.Rate(hz)  # Adjust the rate based on your requirements
    time.sleep(2)
    for i in range(300):
        print(i)
        local_error = calculate_lateral_error(cur_pose, local_des_poses)
        global_error = calculate_lateral_error(cur_pose, global_des_poses)

        # Filter out extreme outliers
        max_local_error = 1.0  # Set the maximum allowed lateral error
        max_global_error = 5.0 

        if local_error <= max_local_error:
            local_errors.append(local_error)
            local_error_publisher.publish(local_error)
            bag.write('/mirte/local_trajectory_error', Float32(local_error), rospy.Time.now())
        else:  
            local_errors.append(local_errors[-1])

        if global_error <= max_global_error:
            global_errors.append(global_error)
            global_error_publisher.publish(global_error)
            bag.write('/mirte/global_trajectory_error', Float32(global_error), rospy.Time.now())
        else:  
            global_errors.append(global_errors[-1])
        rate.sleep()

    # Create a time axis for plotting
    time = range(max(len(local_errors), len(global_errors)))

    # Plot the data
    plt.plot(np.array(time) / hz, local_errors, label="Local Trajectory")
    plt.plot(np.array(time) / hz    , global_errors, label="Global Trajectory")
    plt.xlabel("Time [s]")
    plt.ylabel("Absolute deviation from planned trajectory [m]")
    plt.legend()
    plt.show()

bag.close()  # Close the bag file
