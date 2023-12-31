#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <ros/console.h>

class Localizer{
    public:
        Localizer(ros::NodeHandle& nh);

        void OdomCallback(const nav_msgs::OdometryConstPtr& msg);

        void transformPublisher(const ros::TimerEvent& event);

        private:
            tf::Transform transform;
            std::string frame_id;
            std::string child_frame_id;
            tf::TransformBroadcaster br;
            ros::Timer timer;
            ros::NodeHandle nh_;
            ros::Subscriber odom_sub;
};

Localizer::Localizer(ros::NodeHandle& nh):nh_(nh){
    // br = std::make_shared<tf::TransformBroadcaster>(nh);
    frame_id="";
    odom_sub = nh_.subscribe("/mirte/odom", 10, &Localizer::OdomCallback, this);

    timer = nh_.createTimer(ros::Duration(0.01), &Localizer::transformPublisher, this);

}

void Localizer::OdomCallback(const nav_msgs::OdometryConstPtr& msg){
    frame_id = msg->header.frame_id;
    // int end = frame_id.find("/");
    // child_frame_id = "odom_test";

    geometry_msgs::Point position = msg->pose.pose.position;
    transform.setOrigin(tf::Vector3(position.x, position.y, position.z));
    
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;    
    transform.setRotation(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));

    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, tf_prefix ));
}

void Localizer::transformPublisher(const ros::TimerEvent& event){
    if (frame_id!=""){
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "mirte_tf/map", "mirte_tf/odom"));}
    // ROS_INFO("dfg %s", frame_id);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;
    Localizer fake_localizer(nh); 
    ros::spin();
    return 0;
};