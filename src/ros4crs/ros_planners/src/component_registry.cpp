#include "ros_planners/component_registry.h"

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#ifdef pacejka_model_FOUND
#endif

#include <crs_msgs/car_state_cart.h>
#include <crs_msgs/car_input.h>

#include <planning/cartesian_reference_point.h>
#include <ros_planners/ros_planner.h>
#include <ros_planners/ros_multi_agent_planner.h>
#include <circle_planner/pacejka_circle_planner.h>
#include <circle_planner/multi_pacejka_circle_planner.h>
#include <lloyd_planner/multi_pacejka_lloyd_planner.h>

/**
 * @brief This file loads the specific controller implementation and wraps it inside a ros controller object
 *
 */
namespace ros_planner
{

template <>
ros_planner::RosPlanner<crs_msgs::car_state_cart, crs_models::pacejka_model::pacejka_car_state,
                        crs_planning::cartesian_reference_point>*
resolvePlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& planner_type)
{
  // Create Planner
  auto planner_ptr = std::make_shared<crs_planning::PacejkaCirclePlanner>();
  // Downcast to BasePlanner type
  auto derived_ptr = std::dynamic_pointer_cast<
      crs_planning::BasePlanner<crs_planning::cartesian_reference_point, crs_models::pacejka_model::pacejka_car_state>>(
      planner_ptr);

  return new ros_planner::RosPlanner<crs_msgs::car_state_cart, crs_models::pacejka_model::pacejka_car_state,
                                     crs_planning::cartesian_reference_point>(nh, nh_private, derived_ptr);
}

template <>
ros_planner::RosMultiAgentPlanner<crs_msgs::car_state_cart, crs_models::pacejka_model::pacejka_car_state,
                                  crs_planning::multi_car_cartesian_reference_point>*
resolveMultiAgentPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& planner_type)
{
  // Create Planner
  auto planner_ptr = std::make_shared<crs_planning::MultiPacejkaCirclePlanner>();
  // Downcast to BasePlanner type
  auto derived_ptr = std::dynamic_pointer_cast<
      crs_planning::BasePlanner<crs_planning::multi_car_cartesian_reference_point,
                                std::map<std::string, crs_models::pacejka_model::pacejka_car_state>>>(planner_ptr);

  return new ros_planner::RosMultiAgentPlanner<crs_msgs::car_state_cart, crs_models::pacejka_model::pacejka_car_state,
                                               crs_planning::multi_car_cartesian_reference_point>(nh, nh_private,
                                                                                                  derived_ptr);
};

template <>
ros_planner::RosMultiAgentPlanner<crs_msgs::car_state_cart, crs_models::pacejka_model::pacejka_car_state,
                                  crs_planning::multi_car_cartesian_reference_point>*
resolveMultiAgentLloydPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& planner_type)
{
  // Create Planner
  auto planner_ptr = std::make_shared<crs_planning::MultiPacejkaLloydPlanner>();
  // Downcast to BasePlanner type
  auto derived_ptr = std::dynamic_pointer_cast<
      crs_planning::BasePlanner<crs_planning::multi_car_cartesian_reference_point,
                                std::map<std::string, crs_models::pacejka_model::pacejka_car_state>>>(planner_ptr);

  return new ros_planner::RosMultiAgentPlanner<crs_msgs::car_state_cart, crs_models::pacejka_model::pacejka_car_state,
                                               crs_planning::multi_car_cartesian_reference_point>(nh, nh_private,
                                                                                                  derived_ptr);
};

}  // namespace ros_planner
