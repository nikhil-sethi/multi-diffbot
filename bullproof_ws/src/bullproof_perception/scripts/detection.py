#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Pose, Pose2D, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

farmer_pose = Pose2D()
bull_pose = Pose2D()
mirte_pose = Pose2D()

def get_yaw (msg):
    orientation_q = msg.pose.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = euler_from_quaternion (orientation_list)
    return yaw

def callback_april_detections(msg):
    bull_pose.x = msg.detections[0].pose.pose.pose.position.x
    bull_pose.y = msg.detections[0].pose.pose.pose.position.y
    farmer_pose.x = msg.detections[1].pose.pose.pose.position.x
    farmer_pose.y = msg.detections[1].pose.pose.pose.position.y
    mirte_pose.x = msg.detections[2].pose.pose.pose.position.x
    mirte_pose.y = msg.detections[2].pose.pose.pose.position.y

    bull_pose.theta = get_yaw(msg.detections[0])
    farmer_pose.theta = get_yaw(msg.detections[1])
    mirte_pose.theta = get_yaw(msg.detections[2])
    print('hoi')

    locations_farmer_pub.publish(farmer_pose)
    locations_mirte_pub.publish(mirte_pose)
    locations_bull_pub.publish(bull_pose)



def get_detections():
    rospy.init_node('get_detections_py', anonymous=True)
    print('hoi1')
    locations_farmer_pub = rospy.Publisher('locations/farmer', Pose2D, queue_size=1)
    locations_mirte_pub = rospy.Publisher('locations/mirte', Pose2D, queue_size=1)
    locations_bull_pub = rospy.Publisher('locations/bull', Pose2D, queue_size=1)
    while not rospy.is_shutdown():
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, callback_april_detections)
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        get_detections()
    except rospy.ROSInterruptException:
        pass