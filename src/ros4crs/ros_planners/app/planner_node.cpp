#include "ros_planners/component_registry.h"
#include <planning/cartesian_reference_point.h>
#include <planning/multi_car_cartesian_reference_point.h>
#include <ros/ros.h>

#ifdef pacejka_model_FOUND
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#endif
#include <crs_msgs/car_input.h>
#include <crs_msgs/car_state_cart.h>

// Required reference so object does not go out of scope and gets dereferenced
void* planner_ptr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_planner");
  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/ros_planner/*

  std::string planner_type;
  std::string state_type;
  nh_private.getParam("planner_type", planner_type);  // UNUSED CURRENTLY
  nh_private.getParam("state_type", state_type);

#ifdef pacejka_model_FOUND
  if (state_type == "pacejka_car")
  {
    if (planner_type == "multi_agent_circular_planner")
    {
      planner_ptr = ((void*)ros_planner::resolveMultiAgentPlanner<crs_msgs::car_state_cart,
                                                                  crs_models::pacejka_model::pacejka_car_state,
                                                                  crs_planning::multi_car_cartesian_reference_point>(
          nh, nh_private, planner_type));
    }
    else if (planner_type == "circular_planner")
    {
      planner_ptr =
          ((void*)ros_planner::resolvePlanner<crs_msgs::car_state_cart, crs_models::pacejka_model::pacejka_car_state,
                                              crs_planning::cartesian_reference_point>(nh, nh_private, planner_type));
    }
    else if (planner_type == "multi_agent_lloyd_planner")
    {
      ROS_INFO("try to load multi agent lloyd planner");
      planner_ptr = ((void*)ros_planner::resolveMultiAgentLloydPlanner<crs_msgs::car_state_cart,
                                                                       crs_models::pacejka_model::pacejka_car_state,
                                                                       crs_planning::multi_car_cartesian_reference_point>(
              nh, nh_private, planner_type));
      ROS_INFO("loaded multi agent lloyd planner");
    }

    if (!planner_ptr)
    {
      ROS_ERROR_STREAM("Did not find planner for type: " << state_type << "aborting!");
      return 1;
    }
  }
#endif

  if (!planner_ptr)
  {
    ROS_ERROR_STREAM("Unknown state type: " << state_type << ". Aborting!");
    return 1;
  }
  ros::MultiThreadedSpinner spinner(1);  // Use 1 threads
  spinner.spin();                        // spin() will not return until the node has been shutdown
  return 0;
}
