#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, PoseStamped, TwistWithCovariance


class AprilTagConverter:
    def __init__(self):
        rospy.init_node('april_tag_converter')

        self.window_size = 5
        self.sliding_windows = {1: [], 2: [], 3: []}
        self.odometry_pubs = {
            1: rospy.Publisher('/bull/odom', Odometry, queue_size=5),
            2: rospy.Publisher('/farmer/odom', Odometry, queue_size=5),
            3: rospy.Publisher('/mirte/odom', Odometry, queue_size=5)
        }

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.april_tag_callback)

    def april_tag_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id[0]
            posecovstamped = detection.pose
            posecov = posecovstamped.pose
            pose = posecov.pose
            point = pose.position

            try:
                # Create a new odometry message
                odometry_msg = Odometry()
                odometry_msg.header = posecovstamped.header
                odometry_msg.child_frame_id = posecovstamped.header.frame_id
                # Estimate the velocities using a sliding window algorithm
                twist = self.estimate_twist(tag_id, posecov)
                odometry_msg.twist = twist

                # Fill in the pose information
                odometry_msg.pose = posecov

                # rospy.logwarn(odometry_msg)

                # Publish the odometry message for the corresponding tag
                self.odometry_pubs[tag_id].publish(odometry_msg)
                # rospy.sleep(1)

            except Exception as e:
                rospy.logwarn("Failed to process the AprilTag detection: {}".format(str(e)))

    def estimate_twist(self, tag_id, pose):
        self.sliding_windows[tag_id].append(pose)

        if len(self.sliding_windows[tag_id]) > self.window_size:
            self.sliding_windows[tag_id].pop(0)

        if len(self.sliding_windows[tag_id]) < 2:
            return TwistWithCovariance()

        window_poses = self.sliding_windows[tag_id]
        vel_x_sum = 0.0
        vel_y_sum = 0.0
        vel_z_sum = 0.0

        for i in range(len(window_poses) - 1):
            vel_x_sum += (window_poses[i + 1].pose.position.x - window_poses[i].pose.position.x)
            vel_y_sum += (window_poses[i + 1].pose.position.y - window_poses[i].pose.position.y)
            vel_z_sum += (window_poses[i + 1].pose.position.z - window_poses[i].pose.position.z)

        vel_x_avg = vel_x_sum / (self.window_size - 1)
        vel_y_avg = vel_y_sum / (self.window_size - 1)
        vel_z_avg = vel_z_sum / (self.window_size - 1)

        twist = TwistWithCovariance()
        twist.twist.linear.x = vel_x_avg
        twist.twist.linear.y = vel_y_avg
        twist.twist.linear.z = vel_z_avg

        return twist


if __name__ == '__main__':
    try:
        converter = AprilTagConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
