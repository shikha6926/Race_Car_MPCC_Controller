#include "ros_backtracker/crash_detector.h"
#include <ros/ros.h>
#include <ros_crs_utils/parameter_io.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_crash_detector");
  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/ros_crash_detector/*
  ros_backtracker::CrashDetector crash_detector(
      nh, nh_private, parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track")));

  ros::MultiThreadedSpinner spinner(1);  // Use 1 threads
  spinner.spin();                        // spin() will not return until the node has been shutdown
  return 0;
}
