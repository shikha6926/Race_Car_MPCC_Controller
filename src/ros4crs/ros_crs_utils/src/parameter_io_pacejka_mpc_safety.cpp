#include <ros_crs_utils/parameter_io.h>
#include <pacejka_mpc_safety_filter/config.h>

namespace parameter_io
{

/**
 * @brief Loads the configuration for the pacejka mpc safety filter
 *
 * @param nh
 * @return crs_safety::pacejka_mpc_safety_config
 */
template <>
crs_safety::pacejka_mpc_safety_config getConfig<crs_safety::pacejka_mpc_safety_config>(const ros::NodeHandle& nh)
{
  crs_safety::pacejka_mpc_safety_config config;
  nh.getParam("dist_targ_multiplier", config.dist_targ_multiplier);
  nh.getParam("min_terminal_dist", config.min_terminal_dist);
  nh.getParam("max_terminal_dist", config.max_terminal_dist);
  nh.getParam("use_torque_filter", config.use_torque_filter);
  nh.getParam("dist_decrement_max", config.dist_decrement_max);
  nh.getParam("solver_type", config.solver_type);
  nh.getParam("loookahead_time", config.loookahead_time);
  nh.getParam("reference_method", config.reference_method);
  nh.getParam("threshold", config.threshold);

  nh.getParam("cost_steer", config.cost_steer);
  nh.getParam("cost_torque", config.cost_torque);
  nh.getParam("cost_delta_torque", config.cost_delta_torque);
  nh.getParam("cost_delta_steer", config.cost_delta_steer);
  return config;
}

}  // namespace parameter_io