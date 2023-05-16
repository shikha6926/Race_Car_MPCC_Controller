#ifndef DYNAMIC_TRAJECTORY_UPDATER
#define DYNAMIC_TRAJECTORY_UPDATER

#include <commons/dynamic_point_trajectory.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PolygonStamped.h>

namespace ros_controllers
{

class DynamicTrajectoryUpdator
{
private:
  std::shared_ptr<crs_controls::DynamicPointTrajectory> trajectory_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber traj_sub_;
  ros::Subscriber vor_edges_sub_;

public:
  DynamicTrajectoryUpdator(ros::NodeHandle nh, ros::NodeHandle nh_private,
                           std::shared_ptr<crs_controls::DynamicPointTrajectory> trajectory);

  void rosTrajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr msg);

  void rosVorEdgesCallback(const geometry_msgs::PolygonStampedPtr msg);
};
}  // namespace ros_controllers

#endif /* DYNAMIC_TRAJECTORY_UPDATER */
