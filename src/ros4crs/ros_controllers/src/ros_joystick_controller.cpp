#include "ros_controllers/ros_joystick_controller.h"
#include <crs_msgs/car_input.h>
namespace ros_controllers
{
JoystickController::JoystickController(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  input_publisher_ = nh_private_.advertise<crs_msgs::car_input>("control_input", 1);

  if (!nh_private_.getParam("controller_params/max_torque", max_torque_))
  {
    ROS_WARN_STREAM("JoystickController: Did not load max_torque");
  }
  if (!nh_private_.getParam("controller_params/max_steer", max_steer_))
  {
    ROS_WARN_STREAM("JoystickController: Did not load max_steer");
  }
  if (!nh_private_.getParam("controller_params/min_torque", min_torque_))
  {
    ROS_WARN_STREAM("JoystickController: Did not load min_torque");
  }

  joy_sub_ = nh_.subscribe("joy", 1, &JoystickController::joystickCallback, this);
}

void JoystickController::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  double right_trigger = joy->axes[5];
  double left_trigger = joy->axes[2];
  double joy_steer = joy->axes[0];

  crs_msgs::car_input joystick_input;
  double torque_conversion = -max_torque_ / 2.0 * (right_trigger - 1.0);
  joystick_input.torque = std::max(min_torque_, torque_conversion);
  joystick_input.steer = max_steer_ * joy_steer;
  joystick_input.header.stamp = ros::Time::now();
  joystick_input.header.frame_id = "crs_frame";

  // publish command
  input_publisher_.publish(joystick_input);
}
}  // namespace ros_controllers