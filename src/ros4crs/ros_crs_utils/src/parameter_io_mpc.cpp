#include <ros_crs_utils/parameter_io.h>

#ifdef pacejka_model_FOUND
#include <mpc_controller/pacejka_controller/mpcc_pacejka_config.h>
#include <mpc_controller/pacejka_controller/tracking_mpc_pacejka_config.h>
#endif

namespace parameter_io
{
#ifdef pacejka_model_FOUND
template <>
crs_controls::mpcc_pacejka_config getConfig<crs_controls::mpcc_pacejka_config>(const ros::NodeHandle& nh)
{
  crs_controls::mpcc_pacejka_config params;

  if (!nh.getParam("Q1", params.Q1))
    ROS_WARN_STREAM(" getConfig<crs_controls::mpcc_pacejka_config>: did not load Q1");
  if (!nh.getParam("Q2", params.Q2))
    ROS_WARN_STREAM(" getConfig<crs_controls::mpcc_pacejka_config>: did not load Q2");
  if (!nh.getParam("R1", params.R1))
    ROS_WARN_STREAM(" getConfig<crs_controls::mpcc_pacejka_config>: did not load R1");
  if (!nh.getParam("R2", params.R2))
    ROS_WARN_STREAM(" getConfig<crs_controls::mpcc_pacejka_config>: did not load R2");
  if (!nh.getParam("R3", params.R3))
    ROS_WARN_STREAM(" getConfig<crs_controls::mpcc_pacejka_config>: did not load R3");
  if (!nh.getParam("q", params.q))
    ROS_WARN_STREAM(" getConfig<crs_controls::mpcc_pacejka_config>: did not load q");
  if (!nh.getParam("lag_compensation_time", params.lag_compensation_time))
    ROS_WARN_STREAM(" getConfig<crs_controls::mpcc_pacejka_config>: did not load lag_compensation_time");
  if (!nh.getParam("solver_type", params.solver_type))
    ROS_WARN_STREAM(" getConfig<crs_controls::mpcc_pacejka_config>: did not load solver_type");

  return params;
}

template <>
crs_controls::tracking_mpc_pacejka_config getConfig<crs_controls::tracking_mpc_pacejka_config>(const ros::NodeHandle& nh)
{
  crs_controls::tracking_mpc_pacejka_config params;

  if (!nh.getParam("Q1", params.Q1))
    ROS_WARN_STREAM(" getConfig<crs_controls::tracking_mpc_pacejka_config>: did not load Q1");
  if (!nh.getParam("Q2", params.Q2))
    ROS_WARN_STREAM(" getConfig<crs_controls::tracking_mpc_pacejka_config>: did not load Q2");
  if (!nh.getParam("R1", params.R1))
    ROS_WARN_STREAM(" getConfig<crs_controls::tracking_mpc_pacejka_config>: did not load R1");
  if (!nh.getParam("R2", params.R2))
    ROS_WARN_STREAM(" getConfig<crs_controls::tracking_mpc_pacejka_config>: did not load R2");
  if (!nh.getParam("lag_compensation_time", params.lag_compensation_time))
    ROS_WARN_STREAM(" getConfig<crs_controls::tracking_mpc_pacejka_config>: did not load lag_compensation_time");
  if (!nh.getParam("solver_type", params.solver_type))
    ROS_WARN_STREAM(" getConfig<crs_controls::tracking_mpc_pacejka_config>: did not load solver_type");

  return params;
}

#endif

}  // namespace parameter_io
