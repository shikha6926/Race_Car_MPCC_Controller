#include "ros_safety_framework/component_registry.h"
#include <ros/ros.h>
#include <ros_crs_utils/parameter_io.h>

#include <commons/dynamic_point_trajectory.h>

#define pacejka_model_FOUND

#ifdef pacejka_model_FOUND
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#endif
#define pid_controller_FOUND

#ifdef pacejka_model_FOUND
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_params.h>
#endif

#include <crs_msgs/car_state_cart.h>
#include <crs_msgs/car_input.h>
#include <commons/trajectory.h>

#include <collision_avoider/simple_pacejka_collision_avoider.h>
#include <pacejka_mpc_safety_filter/pacejka_mpc_safety_filter.h>

#include "ros_safety_framework/mpc_safety_visualizer.h"

namespace ros_safety
{

template <>
SafetyFilter<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input,
             crs_msgs::car_state_cart, crs_msgs::car_input>*
resolverSafetyFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& filter_type, void* visualizer)
{
  // Load model from params
  crs_models::pacejka_model::pacejka_params pacejka_params;
  // First load generic, gt model
  parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(ros::NodeHandle(nh, "model/model_params"),
                                                                          pacejka_params);
  // Patch certain params
  parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(
      ros::NodeHandle(nh_private, "model/model_params"), pacejka_params, false);

  // Load model from params
  std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> pacejka_model =
      std::make_shared<crs_models::pacejka_model::DiscretePacejkaModel>(pacejka_params);

  if (filter_type == "simple_collision_avoider")
  {
    // Create Controller
    // TODO, track generation currently static track reference
    auto ptr = std::make_shared<crs_safety::SimplePacejkaCollisionAvoider>(
        parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track")), pacejka_model);

    // Downcast to BaseController type
    auto derived_ptr =
        std::dynamic_pointer_cast<crs_safety::SafetyFilter<crs_models::pacejka_model::pacejka_car_state,
                                                           crs_models::pacejka_model::pacejka_car_input>>(ptr);

    return new SafetyFilter<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input,
                            crs_msgs::car_state_cart, crs_msgs::car_input>(nh, nh_private, derived_ptr);
  }
  else if (filter_type == "mpc_filter")
  {
    // Create Controller
    // TODO, track generation currently static track reference
    auto ptr = std::make_shared<crs_safety::PacejkaMpcSafetyFilter>(
        parameter_io::getConfig<crs_safety::pacejka_mpc_safety_config>(ros::NodeHandle(nh_private, "config")),
        parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track")), pacejka_model);

    // Downcast to BaseSafetyFilter type
    auto derived_ptr =
        std::dynamic_pointer_cast<crs_safety::SafetyFilter<crs_models::pacejka_model::pacejka_car_state,
                                                           crs_models::pacejka_model::pacejka_car_input>>(ptr);

    visualizer = new ros_safety::MpcSafetyVisualizer(ros::NodeHandle(nh_private, "visualizer"), ptr);

    return new SafetyFilter<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input,
                            crs_msgs::car_state_cart, crs_msgs::car_input>(nh, nh_private, derived_ptr);
  }

  return nullptr;
}

}  // namespace ros_safety
