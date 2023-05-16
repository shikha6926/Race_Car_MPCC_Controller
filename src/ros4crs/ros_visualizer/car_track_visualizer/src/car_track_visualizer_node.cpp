#include "crs_msgs/car_com.h"
#include "car_track_visualizer/CarTrackVisualizer.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

CarTrackVisualizer* visualizer;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_track_visualizer");

  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/car_track_visualizer/*

  visualizer = new CarTrackVisualizer(nh, nh_private);

  while (ros::ok())
  {
    ros::spinOnce();
    visualizer->run();
    ros::Duration(1 / visualizer->getNodeRate()).sleep();
  }
  return 0;
}
