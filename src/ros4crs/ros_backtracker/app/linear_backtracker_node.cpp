#include "ros_backtracker/linear_backtracker.h"
#include <ros/ros.h>
#include <ros_crs_utils/parameter_io.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_linear_backtracker_node");
  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/ros_linear_backtracker_node/*
  ros_backtracker::LinearBackTracker backtracker(nh, nh_private);

  ros::MultiThreadedSpinner spinner(1);  // Use 1 threads
  spinner.spin();                        // spin() will not return until the node has been shutdown
  return 0;
}
