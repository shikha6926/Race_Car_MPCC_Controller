#include <Eigen/Core>
#include <vector>

#include <dynamic_models/utils/data_conversion.h>
#include "custom_model_template/custom_input.h"
#include "custom_model_template/custom_state.h"

namespace commons
{

template <>
std::vector<double*>
convertToVector<crs_models::custom_model::custom_state>(crs_models::custom_model::custom_state& state)
{
  return { &state.pos_x, &state.pos_y, &state.yaw, &state.vel_x, &state.vel_y, &state.yaw_rate };
}
template <>

std::vector<const double*>
convertToConstVector<crs_models::custom_model::custom_state>(const crs_models::custom_model::custom_state& state)
{
  return { &state.pos_x, &state.pos_y, &state.yaw, &state.vel_x, &state.vel_y, &state.yaw_rate };
}
template <>

std::vector<double*>
convertToVector<crs_models::custom_model::custom_input>(crs_models::custom_model::custom_input& input)
{
  return { &input.torque, &input.steer };
}
template <>

std::vector<const double*>
convertToConstVector<crs_models::custom_model::custom_input>(const crs_models::custom_model::custom_input& input)
{
  return { &input.torque, &input.steer };
}

template <>
crs_models::custom_model::custom_state
convertToState<crs_models::custom_model::custom_state, 6>(const Eigen::Matrix<double, 6, 1>& vector)
{
  crs_models::custom_model::custom_state state;
  state.pos_x = vector(0, 0);
  state.pos_y = vector(1, 1);
  state.yaw = vector(2, 2);
  state.vel_x = vector(3, 3);
  state.vel_y = vector(4, 4);
  state.yaw_rate = vector(5, 5);
  return state;
}

template <>
Eigen::Matrix<double, 6, 1>
convertToEigen<crs_models::custom_model::custom_state, 6>(const crs_models::custom_model::custom_state& state)
{
  Eigen::Matrix<double, 6, 1> matrix;
  matrix(0, 0) = state.pos_x;
  matrix(1, 0) = state.pos_y;
  matrix(2, 0) = state.yaw;
  matrix(3, 0) = state.vel_x;
  matrix(4, 0) = state.vel_y;
  matrix(5, 0) = state.yaw_rate;
  return matrix;
}

template <>
crs_models::custom_model::custom_state
convertToState<crs_models::custom_model::custom_state, -1>(const Eigen::Matrix<double, -1, 1>& vector)
{
  return convertToState<crs_models::custom_model::custom_state, 6>(vector);
}

template <>
Eigen::Matrix<double, -1, 1>
convertToEigen<crs_models::custom_model::custom_state, -1>(const crs_models::custom_model::custom_state& state)
{
  return convertToEigen<crs_models::custom_model::custom_state, 6>(state);
}
}  // namespace commons