//https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot
// Roughlt based on:
// https://github.com/eborghi10/my_ROS_mobile_robot/blob/master/my_robot_base/include/my_robot_hw_interface.h
// https://github.com/PickNikRobotics/ros_control_boilerplate
// https://github.com/DeborggraeveR/ampru

// https://github.com/resibots/dynamixel_control_hw/blob/master/include/dynamixel_control_hw/hardware_interface.hpp
// https://github.com/FRC900/2018RobotCode/blob/master/zebROS_ws/src/ros_control_boilerplate/include/ros_control_boilerplate/frcrobot_hw_interface.h
// INTERESTING CLEAN ONE: https://github.com/ros-controls/ros_controllers/blob/indigo-devel/diff_drive_controller/test/diffbot.h


#pragma once
#define _USE_MATH_DEFINES

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <mirte_msgs/Encoder.h>
#include <mirte_msgs/SetMotorSpeed.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// ostringstream
#include <sstream>
#include <algorithm>
#include <cmath>

const unsigned int NUM_JOINTS = 2;

/// \brief Hardware interface for a robot
class MyRobotHWInterface : public hardware_interface::RobotHW
{
public:
  MyRobotHWInterface();

  /*
   *
   */
  void write() {
    if (running_){
      // cmd[0] = ros_control calculated speed of left motor in rad/s
      // cmd[1] = ros_control calculated speed of right motor in rad/s

      // This function converts cmd[0] to pwm and calls that service

      //NOTE: this *highly* depends on teh voltage of the motors!!!!
      //For 5V power bank: 255 pwm = 90 ticks/sec -> ca 2 omwenteling/s (4*pi)
      //For 6V power supply: 255 pwm = 120 ticks/sec -> ca 3 omwentelingen/s (6*pi)

      int left_speed  = std::max(std::min(int(cmd[0] / ( 6 * M_PI) * 100), 100), -100);
      if (left_speed != _last_cmd[0]){
        left_motor_service.request.speed = left_speed;
        left_client.call(left_motor_service);
        _last_cmd[0] = left_speed;
      }

      int right_speed = std::max(std::min(int(cmd[1] / ( 6 * M_PI) * 100), 100), -100);
      if (right_speed != _last_cmd[1]){
        right_motor_service.request.speed = right_speed;
        right_client.call(right_motor_service);
        _last_cmd[1] = right_speed;
      }
      // Set the direction in so the read() can use it
      // TODO: this does not work propely, because at the end of a series cmd_vel is negative, while the rotation is not
      _last_wheel_cmd_direction[0] = cmd[0] > 0.0 ? 1 : -1;
      _last_wheel_cmd_direction[1] = cmd[1] > 0.0 ? 1 : -1;

    }
 }

  /**
   * Reading encoder values and setting position and velocity of enconders
   */
  void read(const ros::Duration &period) {
    //_wheel_encoder[0] = number of ticks of left encoder since last call of this function
    //_wheel_encoder[1] = number of ticks of right encoder since last call of this function

    double meterPerEncoderTick = (_wheel_diameter / 2) * 2 * M_PI / 40.0;
    int diff_left  = _wheel_encoder[0] - _last_value[0];
    int diff_right = _wheel_encoder[1] - _last_value[1];
    _last_value[0] = _wheel_encoder[0];
    _last_value[1] = _wheel_encoder[1];

    double magic_number = 1.0;

    double distance_left  = diff_left * meterPerEncoderTick * _last_wheel_cmd_direction[0] * magic_number;
    double distance_right = diff_right * meterPerEncoderTick * _last_wheel_cmd_direction[1] * magic_number;

    pos[0] += distance_left;
//    vel[0] = distance_left / period.toSec();
    pos[1] += distance_right;
//    vel[1] = distance_right / period.toSec();

  }

/*
  ros::Time get_time() {
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
  }

  ros::Duration get_period() {
    return curr_update_time - prev_update_time;
  }
*/
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];

  bool running_;
  double _wheel_diameter;
  double _max_speed;
  double _wheel_angle[NUM_JOINTS];
  int _wheel_encoder[NUM_JOINTS];
  int _last_cmd[NUM_JOINTS];
  int _last_value[NUM_JOINTS];
  int _last_wheel_cmd_direction[NUM_JOINTS];

  ros::Time curr_update_time, prev_update_time;

  ros::Subscriber left_wheel_encoder_sub_;
  ros::Subscriber right_wheel_encoder_sub_;

  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  ros::ServiceClient left_client;
  ros::ServiceClient right_client;


  mirte_msgs::SetMotorSpeed left_motor_service;
  mirte_msgs::SetMotorSpeed right_motor_service;

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }


  void leftWheelEncoderCallback(const mirte_msgs::Encoder& msg) {
    _wheel_encoder[0] = _wheel_encoder[0] + msg.value;
  }

  void rightWheelEncoderCallback(const mirte_msgs::Encoder& msg) {
    _wheel_encoder[1] = _wheel_encoder[1] + msg.value;
  }

};  // class

MyRobotHWInterface::MyRobotHWInterface()
: running_(true)
  , private_nh("~")
  , start_srv_(nh.advertiseService("start", &MyRobotHWInterface::start_callback, this))
  , stop_srv_(nh.advertiseService("stop", &MyRobotHWInterface::stop_callback, this)) 
  {
    private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.06);
    private_nh.param<double>("max_speed", _max_speed, 2.0);

    // Intialize raw data
    std::fill_n(pos, NUM_JOINTS, 0.0);
    std::fill_n(vel, NUM_JOINTS, 0.0);
    std::fill_n(eff, NUM_JOINTS, 0.0);
    std::fill_n(cmd, NUM_JOINTS, 0.0);

    // connect and register the joint state and velocity interfaces
    for (unsigned int i = 0; i < NUM_JOINTS; ++i)
    {
      std::ostringstream os;
      os << "wheel_" << i << "_joint";

      hardware_interface::JointStateHandle state_handle(os.str(), &pos[i], &vel[i], &eff[i]);
      jnt_state_interface.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(os.str()), &cmd[i]);
      jnt_vel_interface.registerHandle(vel_handle);

      _wheel_encoder[i] = 0;
      _last_value[i] = 0;
      _last_wheel_cmd_direction[i] = 0;
      _last_cmd[i] = 0;
    }
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_vel_interface);

    // Initialize publishers and subscribers
    left_wheel_encoder_sub_ = nh.subscribe("/mirte/encoder/left", 1, &MyRobotHWInterface::leftWheelEncoderCallback, this);
    right_wheel_encoder_sub_ = nh.subscribe("/mirte/encoder/right", 1, &MyRobotHWInterface::rightWheelEncoderCallback, this);


    ros::service::waitForService("/mirte/set_left_speed");
    ros::service::waitForService("/mirte/set_right_speed");
    left_client = nh.serviceClient<mirte_msgs::SetMotorSpeed>("/mirte/set_left_speed", true);
    right_client = nh.serviceClient<mirte_msgs::SetMotorSpeed>("/mirte/set_right_speed", true);
    // TODO: checl ion isvalis when running https://answers.ros.org/question/281411/persistent-service-initialization/
}
