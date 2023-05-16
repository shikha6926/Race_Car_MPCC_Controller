#ifndef SRC_ROS_ROS_PLANNERS_INCLUDE_ROS_PLANNERS_COMPONENT_REGISTRY
#define SRC_ROS_ROS_PLANNERS_INCLUDE_ROS_PLANNERS_COMPONENT_REGISTRY
#include "ros_planners/ros_planner.h"
#include "ros_planners/ros_multi_agent_planner.h"
#include "ros/ros.h"
namespace ros_planner
{

/**
 * @brief Returns the ros_controller for the requested types.
 *
 * @tparam StateMsg State Message Type (ros)
 * @tparam InputMsg  Input Message Type (ros)
 * @tparam StateType  State Message Type (crs)
 * @tparam InputType  Input Message Type (crs)
 * @param nh  public nodehandle
 * @param nh_private  private nodehandle
 * @param controller_type type of the controller (e.g. pid, mpcc,... )
 * @param dynamic_callback_allocator  pointer to memory where the reference for the dynamic reconfigure callback is
 * stored, if dynamic configure is defined
 * @return RosController<StateMsg, InputMsg, StateType, InputType>*
 */
template <typename StateMsg, typename StateType, typename TrajectoryType>
ros_planner::RosPlanner<StateMsg, StateType, TrajectoryType>*
resolvePlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& planner_type);

template <typename StateMsg, typename StateType, typename TrajectoryType>
ros_planner::RosMultiAgentPlanner<StateMsg, StateType, TrajectoryType>*
resolveMultiAgentPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& planner_type);

template <typename StateMsg, typename StateType, typename TrajectoryType>
ros_planner::RosMultiAgentPlanner<StateMsg, StateType, TrajectoryType>*
resolveMultiAgentLloydPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& planner_type);

}  // namespace ros_planner
#endif /* SRC_ROS_ROS_PLANNERS_INCLUDE_ROS_PLANNERS_COMPONENT_REGISTRY */
