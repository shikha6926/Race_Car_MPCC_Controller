#include <Eigen/Core>
#include <vector>

#include <dynamic_models/utils/data_conversion.h>
#include <kinematic_model/kinematic_car_input.h>
#include <kinematic_model/kinematic_car_state.h>

namespace commons
{
template <>
std::vector<double*> convertToVector<crs_models::kinematic_model::kinematic_car_state>(
    crs_models::kinematic_model::kinematic_car_state& state)
{
  return { &state.pos_x, &state.pos_y, &state.yaw, &state.velocity };
}

template <>
std::vector<const double*> convertToConstVector<crs_models::kinematic_model::kinematic_car_state>(
    const crs_models::kinematic_model::kinematic_car_state& state)
{
  return { &state.pos_x, &state.pos_y, &state.yaw, &state.velocity };
}

template <>
std::vector<double*> convertToVector<crs_models::kinematic_model::kinematic_car_input>(
    crs_models::kinematic_model::kinematic_car_input& input)
{
  return { &input.torque, &input.steer };
}

template <>
std::vector<const double*> convertToConstVector<crs_models::kinematic_model::kinematic_car_input>(
    const crs_models::kinematic_model::kinematic_car_input& input)
{
  return { &input.torque, &input.steer };
}

template <>
crs_models::kinematic_model::kinematic_car_state
convertToState<crs_models::kinematic_model::kinematic_car_state, -1>(const Eigen::Matrix<double, -1, 1>& vector)
{
  crs_models::kinematic_model::kinematic_car_state state;
  state.pos_x = vector(0, 0);
  state.pos_y = vector(1, 1);
  state.yaw = vector(2, 2);
  state.velocity = vector(3, 3);
  return state;
}

template <>
crs_models::kinematic_model::kinematic_car_state
convertToState<crs_models::kinematic_model::kinematic_car_state, 4>(const Eigen::Matrix<double, 4, 1>& vector)
{
  return convertToState<crs_models::kinematic_model::kinematic_car_state, -1>(vector);
}

template <>
Eigen::Matrix<double, 4, 1> convertToEigen<crs_models::kinematic_model::kinematic_car_state, 4>(
    const crs_models::kinematic_model::kinematic_car_state& state)
{
  Eigen::Matrix<double, 4, 1> matrix;
  matrix(0, 0) = state.pos_x;
  matrix(1, 0) = state.pos_y;
  matrix(2, 0) = state.yaw;
  matrix(3, 0) = state.velocity;
  return matrix;
}

template <>
Eigen::Matrix<double, -1, 1> convertToEigen<crs_models::kinematic_model::kinematic_car_state, -1>(
    const crs_models::kinematic_model::kinematic_car_state& state)
{
  return convertToEigen<crs_models::kinematic_model::kinematic_car_state, 4>(state);
}
}  // namespace commons