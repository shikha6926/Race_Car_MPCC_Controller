#ifndef SRC_ROS_ROS_SIMULATOR_INCLUDE_ROS_SIMULATOR_COMPONENT_REGISTRY
#define SRC_ROS_ROS_SIMULATOR_INCLUDE_ROS_SIMULATOR_COMPONENT_REGISTRY

#include "ros_simulator/common/ros_simulator.h"
#include <ros/ros.h>

namespace ros_simulator
{
Simulator* resolveSimulator(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& state_type,
                            const std::string& input_type, const std::vector<std::string>& sensors_to_load);
}  // namespace ros_simulator
#endif /* SRC_ROS_ROS_SIMULATOR_INCLUDE_ROS_SIMULATOR_COMPONENT_REGISTRY */
