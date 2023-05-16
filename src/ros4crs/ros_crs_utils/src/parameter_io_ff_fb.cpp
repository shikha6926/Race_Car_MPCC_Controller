#include <ros_crs_utils/parameter_io.h>
#include <ff_fb_controller/ff_fb_config.h>

namespace parameter_io
{

template <>
crs_controls::FfFbConfig getConfig<crs_controls::FfFbConfig>(const ros::NodeHandle& nh)
{
  crs_controls::FfFbConfig params;

  if (!nh.getParam("target_velocity", params.target_velocity))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load target_velocity");
  if (!nh.getParam("lag_compensation_time", params.lag_compensation_time))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load lag_compensation_time");
  if (!nh.getParam("b_filter", params.b_filter))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load b_filter");
  if (!nh.getParam("a_filter", params.a_filter))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load a_filter");
  if (!nh.getParam("a_torque", params.a_torque))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load a_torque");
  if (!nh.getParam("b_torque", params.b_torque))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load b_torque");
  if (!nh.getParam("Kd", params.Kd))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load Kd");
  if (!nh.getParam("Kp", params.Kp))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load Kp");
  if (!nh.getParam("Ki", params.Ki))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load Ki");
  if (!nh.getParam("steer_limit", params.steer_limit))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load steer_limit");
  if (!nh.getParam("K_torque_curv", params.K_torque_curv))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load K_torque_curv");
  if (!nh.getParam("mean_curv_dist", params.mean_curv_dist))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load mean_curv_dist");
  if (!nh.getParam("look_ahead_dist", params.look_ahead_dist))
    ROS_WARN_STREAM(" getConfig<crs_controls::FfFbConfig>: did not load look_ahead_dist");

  return params;
}

}  // namespace parameter_io