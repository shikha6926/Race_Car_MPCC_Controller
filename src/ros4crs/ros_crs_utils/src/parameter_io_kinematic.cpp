#include <ros_crs_utils/parameter_io.h>
#include <kinematic_model/kinematic_params.h>
#include <kinematic_model/kinematic_car_input.h>
#include <kinematic_model/kinematic_car_state.h>
namespace parameter_io
{

template <>
crs_models::kinematic_model::kinematic_car_state getState(const ros::NodeHandle& nh)
{
  crs_models::kinematic_model::kinematic_car_state state;
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
    state.velocity = state_as_vec[3];
  }
  return state;
}

template <>
crs_models::kinematic_model::kinematic_car_input getInput(const ros::NodeHandle& nh)
{
  crs_models::kinematic_model::kinematic_car_input input;
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
void getModelParams<crs_models::kinematic_model::kinematic_params>(
    const ros::NodeHandle& nh, crs_models::kinematic_model::kinematic_params& params, bool verbose)
{
  if (!nh.getParam("lf", params.lf))
    ROS_WARN_STREAM(" getModelParams<crs_models::kinematic_model::kinematic_params>: did not load lf. Namespace: "
                    << nh.getNamespace());
  if (!nh.getParam("lr", params.lr))
    ROS_WARN_STREAM(
        " getModelParams<crs_models::kinematic_model::kinematic_params>: did not load lr. Ns:" << nh.getNamespace());
  if (!nh.getParam("tau", params.tau))
    ROS_WARN_STREAM(
        " getModelParams<crs_models::kinematic_model::kinematic_params>: did not load tau. Ns:" << nh.getNamespace());

  if (!nh.getParam("a", params.a))
    ROS_WARN_STREAM(
        " getModelParams<crs_models::kinematic_model::kinematic_params>: did not load a. Ns:" << nh.getNamespace());
  if (!nh.getParam("b", params.b))
    ROS_WARN_STREAM(
        " getModelParams<crs_models::kinematic_model::kinematic_params>: did not load b. Ns:" << nh.getNamespace());
}

}  // namespace parameter_io