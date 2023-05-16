#include <ros_crs_utils/parameter_io.h>

#include <lowpass_estimator/car_lowpass_parameters.h>
namespace parameter_io
{

template <>
crs_estimators::lowpass_estimator::car_lowpass_parameters
getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters>(const ros::NodeHandle& nh)
{
  crs_estimators::lowpass_estimator::car_lowpass_parameters params;

  if (!nh.getParam("b_dx", params.b_dx))
    ROS_WARN_STREAM(" getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters >: did not load b_dx");
  if (!nh.getParam("a_dx", params.a_dx))
    ROS_WARN_STREAM(" getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters >: did not load a_dx");

  if (!nh.getParam("b_dy", params.b_dy))
    ROS_WARN_STREAM(" getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters >: did not load b_dy");
  if (!nh.getParam("a_dy", params.a_dy))
    ROS_WARN_STREAM(" getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters >: did not load a_dy");

  if (!nh.getParam("b_dyaw", params.b_dyaw))
    ROS_WARN_STREAM(" getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters >: did not load b_dyaw");
  if (!nh.getParam("a_dyaw", params.a_dyaw))
    ROS_WARN_STREAM(" getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters >: did not load a_dyaw");

  if (!nh.getParam("b_yaw", params.b_yaw))
    ROS_WARN_STREAM(" getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters >: did not load b_yaw");
  if (!nh.getParam("a_yaw", params.a_yaw))
    ROS_WARN_STREAM(" getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters >: did not load a_yaw");

  return params;
}

}  // namespace parameter_io