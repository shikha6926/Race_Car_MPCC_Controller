#include "ros_backtracker/input_filter.h"
#include <ros_crs_utils/parameter_io.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_input_filter");
  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/ros_input_filter/*
  ros_backtracker::InputFilter crash_detector(nh, nh_private);

  ros::MultiThreadedSpinner spinner(1);  // Use 1 threads
  spinner.spin();                        // spin() will not return until the node has been shutdown
  return 0;
}
