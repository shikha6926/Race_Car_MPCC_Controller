#include <ros_crs_utils/parameter_io.h>

#include <pacejka_model/pacejka_params.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_car_input.h>

namespace parameter_io
{

template <>
crs_models::pacejka_model::pacejka_car_state getState(const ros::NodeHandle& nh)
{
  crs_models::pacejka_model::pacejka_car_state state;
  std::vector<double> state_as_vec;
  if (!nh.getParam("value", state_as_vec))
  {
    ROS_WARN_STREAM("Could not load initial state from parameters.");
  }
  else
  {
    state.pos_x = state_as_vec[0];
    state.pos_y = state_as_vec[1];
    state.yaw = state_as_vec[2];
    state.vel_x = state_as_vec[3];
    state.vel_y = state_as_vec[4];
    state.yaw_rate = state_as_vec[5];
  }
  return state;
}

template <>
crs_models::pacejka_model::pacejka_car_input getInput(const ros::NodeHandle& nh)
{
  crs_models::pacejka_model::pacejka_car_input input;
  std::vector<double> input_as_vec;
  if (!nh.getParam("value", input_as_vec))  // Load initial input from params
                                            // (pacejka_car_ekf.yaml)
  {
    ROS_WARN_STREAM("Could not load initial input!");
  }
  else
  {
    input.steer = input_as_vec[0];
    input.torque = input_as_vec[1];
  }
  return input;
}

template <>
void getModelParams<>(const ros::NodeHandle& nh, crs_models::pacejka_model::pacejka_params& params,
                      bool verbose /* = true*/)
{
  if (!nh.getParam("lr", params.lr) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load lr. Namespace " << nh.getNamespace());
  if (!nh.getParam("lf", params.lf) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load lf. Namespace " << nh.getNamespace());
  if (!nh.getParam("m", params.m) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load m. Namespace " << nh.getNamespace());
  if (!nh.getParam("I", params.I) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load I. Namespace " << nh.getNamespace());
  if (!nh.getParam("Df", params.Df) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load Df. Namespace " << nh.getNamespace());
  if (!nh.getParam("Cf", params.Cf) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load Cf. Namespace " << nh.getNamespace());
  if (!nh.getParam("Bf", params.Bf) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load Bf. Namespace " << nh.getNamespace());
  if (!nh.getParam("Dr", params.Dr) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load Dr. Namespace " << nh.getNamespace());
  if (!nh.getParam("Cr", params.Cr) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load Cr. Namespace " << nh.getNamespace());
  if (!nh.getParam("Br", params.Br) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load Br. Namespace " << nh.getNamespace());
  if (!nh.getParam("Cm1", params.Cm1) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load Cm1. Namespace " << nh.getNamespace());
  if (!nh.getParam("Cm2", params.Cm2) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load Cm2. Namespace " << nh.getNamespace());
  if (!nh.getParam("Cd", params.Cd) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load Cd. Namespace " << nh.getNamespace());
  if (!nh.getParam("Croll", params.Croll) && verbose)
    ROS_WARN_STREAM(" PacejkaParams: did not load Croll. Namespace " << nh.getNamespace());
}
}  // namespace parameter_io