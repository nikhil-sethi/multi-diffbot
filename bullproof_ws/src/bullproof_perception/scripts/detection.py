#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from apriltag_ros import AprilTagDetectionArray, AprilTagDetection
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


    def get_rotation(msg):
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        print
        yaw


def get_detections():
    rospy.init_node('listener', anonymous=True)
    # create subscriber
    rospy.Subscriber('apriltag_ros/AprilTagDetectionArray', AprilTagDetectionArray,
                     callback_april_detections)

    rospy.spin()

