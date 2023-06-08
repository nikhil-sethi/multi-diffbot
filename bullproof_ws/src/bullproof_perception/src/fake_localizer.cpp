#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <ros/console.h>

class FakeLocalizer{
    public:
        FakeLocalizer(ros::NodeHandle& nh);

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

FakeLocalizer::FakeLocalizer(ros::NodeHandle& nh):nh_(nh){
    // br = std::make_shared<tf::TransformBroadcaster>(nh);
    frame_id="";
    odom_sub = nh_.subscribe("gazebo/odom_gt", 10, &FakeLocalizer::OdomCallback, this);

    timer = nh_.createTimer(ros::Duration(0.01), &FakeLocalizer::transformPublisher, this);

}

void FakeLocalizer::OdomCallback(const nav_msgs::OdometryConstPtr& msg){
    frame_id = msg->header.frame_id;
    int end = frame_id.find("/");
    child_frame_id = frame_id.substr(0, end) + "/base_footprint";

    geometry_msgs::Point position = msg->pose.pose.position;
    transform.setOrigin(tf::Vector3(position.x, position.y, position.z));
    
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;    
    transform.setRotation(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));

    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, tf_prefix ));
}

void FakeLocalizer::transformPublisher(const ros::TimerEvent& event){
    if (frame_id!=""){
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));}
    // ROS_INFO("dfg %s", frame_id);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "fake_localizer");
    ros::NodeHandle nh;
    FakeLocalizer fake_localizer(nh); 
    ros::spin();
    return 0;
};