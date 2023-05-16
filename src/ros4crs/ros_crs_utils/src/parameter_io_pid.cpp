#include <ros_crs_utils/parameter_io.h>

#include <pid_controller/pid_config.h>

namespace parameter_io
{

template <>
crs_controls::pid_config getConfig<crs_controls::pid_config>(const ros::NodeHandle& nh)
{
  crs_controls::pid_config params;
  if (!nh.getParam("target_velocity", params.target_velocity))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load target_velocity");
  if (!nh.getParam("lag_compensation_time", params.lag_compensation_time))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load lag_compensation_time");
  if (!nh.getParam("b_filter", params.b_filter))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load b_filter");
  if (!nh.getParam("a_filter", params.a_filter))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load a_filter");
  if (!nh.getParam("a_torque", params.a_torque))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load a_torque");
  if (!nh.getParam("b_torque", params.b_torque))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load b_torque");
  if (!nh.getParam("Kd", params.Kd))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load Kd");
  if (!nh.getParam("Kp", params.Kp))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load Kp");
  if (!nh.getParam("Ki", params.Ki))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load Ki");
  if (!nh.getParam("use_filter", params.use_filter))
    ROS_WARN_STREAM(" getConfig<crs_controls::use_filter>: did not load use_filter");
  if (!nh.getParam("steer_limit", params.steer_limit))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load steer_limit");
  return params;
}

template <>
crs_controls::pid_const_ref_config getConfig<crs_controls::pid_const_ref_config>(const ros::NodeHandle& nh)
{
  crs_controls::pid_const_ref_config params;
  if (!nh.getParam("lag_compensation_time", params.lag_compensation_time))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load lag_compensation_time");
  if (!nh.getParam("b_filter", params.b_filter))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load b_filter");
  if (!nh.getParam("a_filter", params.a_filter))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load a_filter");
  if (!nh.getParam("a_torque", params.a_torque))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load a_torque");
  if (!nh.getParam("b_torque", params.b_torque))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load b_torque");
  if (!nh.getParam("Kd", params.Kd))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load Kd");
  if (!nh.getParam("Kp", params.Kp))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load Kp");
  if (!nh.getParam("Ki", params.Ki))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load Ki");
  if (!nh.getParam("Kd_angle", params.Kd_angle))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load Kd_angle");
  if (!nh.getParam("Kp_angle", params.Kp_angle))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load Kp_angle");
  if (!nh.getParam("Ki_angle", params.Ki_angle))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load Ki_angle");
  if (!nh.getParam("use_filter", params.use_filter))
    ROS_WARN_STREAM(" getConfig<crs_controls::use_filter>: did not load use_filter");
  if (!nh.getParam("steer_limit", params.steer_limit))
    ROS_WARN_STREAM(" getConfig<crs_controls::pid_config>: did not load steer_limit");
  return params;
}

}  // namespace parameter_io
