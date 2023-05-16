#include "ros_safety_framework/component_registry.h"
#include "ros_safety_framework/safety_filter.h"
#include <ros/ros.h>
#include <ros_crs_utils/parameter_io.h>
#include <crs_msgs/car_input.h>
#include <crs_msgs/car_state_cart.h>
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>

void* safety_filter;
void* vis_ptr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_filter_node");
  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/safety_filter/*

  std::string state_type;
  std::string input_type;
  std::string safety_filter_type;

  nh_private.getParam("state_type", state_type);
  nh_private.getParam("input_type", input_type);
  nh_private.getParam("safety_filter_type", safety_filter_type);
  std::cout << "params: " << state_type << ", " << input_type << " " << safety_filter_type << std::endl;
  //#ifdef pacejka_model_FOUND
  if (state_type == "pacejka_car" && input_type == "pacejka_car")
  {
    safety_filter =
        (void*)ros_safety::resolverSafetyFilter<crs_models::pacejka_model::pacejka_car_state,
                                                crs_models::pacejka_model::pacejka_car_input, crs_msgs::car_state_cart,
                                                crs_msgs::car_input>(nh, nh_private, safety_filter_type, vis_ptr);
  }
  //#endif

  ros::MultiThreadedSpinner spinner(1);  // Use 1 threads
  spinner.spin();                        // spin() will not return until the node has been shutdown
  return 0;
}
