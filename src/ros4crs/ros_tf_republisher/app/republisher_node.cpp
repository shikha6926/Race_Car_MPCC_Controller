#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_tf_republisher");
  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/ros_tf_republisher/*

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("mocap", 1);

  std::string source_frame, target_frame;
  float update_rate = 200.0;

  if (!nh_private.getParam("source_frame", source_frame))
  {
    ROS_ERROR("No source_frame from config, exiting...");
    return -1;
  }

  if (!nh_private.getParam("target_frame", target_frame))
  {
    ROS_ERROR("No target_frame from config, exiting...");
    return -1;
  }

  while (nh.ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      // Perform a blocking wait on the latest transform to become available until timeout.
      // This ensures minimal delay when the transforms become available and appropriately
      // stops publishing if new data stop arriving.
      transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time::now(), ros::Duration(1.0));
      pub.publish(transformStamped);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
  return 0;
}
