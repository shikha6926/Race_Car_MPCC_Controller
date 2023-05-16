#ifndef COMPONENT_REGISTRY
#define COMPONENT_REGISTRY

#include "ros_safety_framework/safety_filter.h"
#include "ros/ros.h"

namespace ros_safety
{

/**
 * @brief Returns the safety filter for the requested types.
 *
 * @tparam StateMsg State Message Type (ros)
 * @tparam InputMsg  Input Message Type (ros)
 * @tparam StateType  State Message Type (crs)
 * @tparam InputType  Input Message Type (crs)
 * @param nh  public nodehandle
 * @param nh_private  private nodehandle
 * @param filter_type type of the filter
 *
 */
template <typename StateMsg, typename InputMsg, typename StateType, typename InputType>
SafetyFilter<StateMsg, InputMsg, StateType, InputType>* resolverSafetyFilter(ros::NodeHandle& nh,
                                                                             ros::NodeHandle& nh_private,
                                                                             const std::string& filter_type,
                                                                             void* visualizer);

}  // namespace ros_safety
#endif /* SRC_ROS_ROS_SAFETY_FRAMEWORK_INCLUDE_ROS_SAFETY_FRAMEWORK_COMPONENT_REGISTRY */
