/**
 * @file    battery_monitor_node.cpp
 * @author  Lukas Vogel
 * @brief   Node that observes the battery topic in the same namespace and issues warnings if the
 *          batteries should be swapped.
 */

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

/* Global variables --------------------------------------------------------- */

// Both these variables are set by config files on node launch.

// Global rate at which battery voltage warnings should maximally be issued at.
float node_rate_ = 1.0;
// Threshold voltage below which voltage warnings should be issued.
float warn_threshold_ = 5.0;

/* Callback function -------------------------------------------------------- */
void batteryStateCallback(const sensor_msgs::BatteryState& msg)
{
  if (msg.voltage < warn_threshold_)
  {
    ROS_WARN_STREAM_THROTTLE(1 / node_rate_, "Low battery voltage detected on " << ros::this_node::getNamespace());
  }
}

/* Entry point -------------------------------------------------------------- */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_battery_monitor");
  ros::NodeHandle nh;                                 // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/ros_battery_monitor_node/*

  nh_private.getParam("rate", node_rate_);
  nh_private.getParam("threshold", warn_threshold_);

  // Set up subscriber to /<NAMESPACE>/battery
  ros::Subscriber sub = nh.subscribe("battery", 10, &batteryStateCallback);
  ros::spin();

  return 0;
}
