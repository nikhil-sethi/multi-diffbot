#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>

void OdomCallback(const nav_msgs::OdometryConstPtr& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    std::string frame_id = msg->header.frame_id;
    int end = frame_id.find("/");
    std::string tf_prefix =frame_id.substr(0, end);

    geometry_msgs::Point position = msg->pose.pose.position;
    transform.setOrigin(tf::Vector3(position.x, position.y, position.z));
    
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;    
    transform.setRotation(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, tf_prefix + "/base_footprint"));

}

int main(int argc, char** argv){
    ros::init(argc, argv, "diffbot_fake_localizer");
    ros::NodeHandle nh;

    ros::Subscriber robot_sub = nh.subscribe("/mirte/gazebo/odom_gt", 10, &OdomCallback);
    ros::Subscriber farmer_sub = nh.subscribe("/farmer/gazebo/odom_gt", 10, &OdomCallback);
    ros::Subscriber bull_sub = nh.subscribe("/bull/gazebo/odom_gt", 10, &OdomCallback);

    ros::spin();
    return 0;
};