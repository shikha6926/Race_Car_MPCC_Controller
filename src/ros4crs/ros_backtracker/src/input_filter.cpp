#include "ros_backtracker/input_filter.h"

namespace ros_backtracker
{
InputFilter::InputFilter(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private)
{
  // How long to wait after the last emergency input to switch back to the main controller input
  nh_private_.getParam("block_time", block_time_);

  input_publisher_ = nh_.advertise<crs_msgs::car_input>("control_input", 10);
  regular_input_sub_ = nh_private_.subscribe("regular_input", 1, &InputFilter::inputCallback, this);
  emergency_input_sub_ = nh_private_.subscribe("emergency_input", 1, &InputFilter::emergencyInputCallback, this);
}

/**
 */
void InputFilter::inputCallback(crs_msgs::car_input::ConstPtr input_msg)
{
  if (input_msg->header.stamp.toSec() - last_emergency_input_ts_ > block_time_)
  {
    input_publisher_.publish(input_msg);
    if (emergency_mode_ == true)
    {  // Swap from emergency back to non emergency happening
      ROS_INFO("Recovered from Emergency input! Will switch back to control input!");
      emergency_mode_ = false;
    }
  }
}
/**
 */
void InputFilter::emergencyInputCallback(crs_msgs::car_input::ConstPtr input_msg)
{
  if (input_msg->header.stamp.toSec() - last_emergency_input_ts_ > block_time_)
  {
    emergency_mode_ = true;
    ROS_INFO("Detected Emergency Input! Will block regular control inputs");
  };

  last_emergency_input_ts_ = input_msg->header.stamp.toSec();
  input_publisher_.publish(input_msg);
}
}  // namespace ros_backtracker
