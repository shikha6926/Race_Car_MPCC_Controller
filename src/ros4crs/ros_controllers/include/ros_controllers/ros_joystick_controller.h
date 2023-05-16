#pragma once
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
namespace ros_controllers
{
class JoystickController
{
private:
  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher input_publisher_;
  ros::Subscriber joy_sub_;
  double min_torque_ = 0.0;
  double max_torque_ = 0.5;
  double max_steer_ = 0.4;

public:
  JoystickController(ros::NodeHandle nh, ros::NodeHandle nh_private);
  // !
  /*
    ROS callback to grab the joystick input from joy message
    Converts joy message into crs commands based on configuration
    max_steer, min_torque, max_torque parameters, then immediately
    publishes.
  */
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
};
}  // namespace ros_controllers
