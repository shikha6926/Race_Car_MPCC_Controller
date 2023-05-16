
#include "pacejka_sensor_model/lighthouse_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{

// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
LighthouseSensorModel::LighthouseSensorModel(const Eigen::Matrix4d& R, Eigen::Vector3d position, Eigen::Matrix3d rotation, double light_plane_tilt, Eigen::Matrix<double, 2, 4> sensor_positions)
  : SensorModel(4, LighthouseSensorModel::SENSOR_KEY)  // Measurement dimension is four
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),   casadi::MX::sym("yaw"),
                                       casadi::MX::sym("v_x"), casadi::MX::sym("v_y"), casadi::MX::sym("yaw_rate") };

  Eigen::Matrix<casadi::MX, 3, 2> rot;
  rot << cos(state_mx[2]), -sin(state_mx[2]), sin(state_mx[2]), cos(state_mx[2]), 0, 0;
  // Position of the car in world frame
  Eigen::Matrix<casadi::MX, 3, 1> pos_car {state_mx[0], state_mx[1], 0};
  // Positions of the sensors in world frame
  Eigen::Matrix<casadi::MX, 3, 4> pos_sensor = (rot * sensor_positions.cast<casadi::MX>()).colwise() + pos_car;
  // Base station rotation matrix and position vector
  Eigen::Matrix<casadi::MX, 3, 3> rot_bs = rotation.transpose().cast<casadi::MX>();
  Eigen::Matrix<casadi::MX, 3, 1> pos_bs = position.cast<casadi::MX>();
  // Positions in base satation reference frame
  Eigen::Matrix<casadi::MX, 3, 4> sbs = rot_bs * (pos_sensor.colwise() - pos_bs);
  // Calculate angles
  Eigen::Matrix<casadi::MX, 1, 4> alphas = (sbs.row(1).array() / sbs.row(0).array()).atan();
  Eigen::Matrix<casadi::MX, 1, 4> alpha = alphas.array() + ((sbs.row(2) * tan(light_plane_tilt)).array() / 
                                          (sbs.row(0).array().square() + sbs.row(1).array().square()).sqrt()).asin();

  std::vector<casadi::MX> measured_states_mx = { alpha(0), alpha(1), alpha(2), alpha(3) };
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);

  R_ = R;
  base_station_position_ = position;
  base_station_rotation_ = rotation;
  light_plane_tilt_ = light_plane_tilt;
  sensor_positions_ = sensor_positions;
}

const std::string LighthouseSensorModel::SENSOR_KEY = "lighthouse";

/**
 * @brief Evaluates the measurement model at the given state
 *
 * @param state
 * @return double vector of measured states (x,y,yaw)
 */
Eigen::Matrix<double, -1, 1> LighthouseSensorModel::applyModel(const crs_models::pacejka_model::pacejka_car_state& state,
                                                          const crs_models::pacejka_model::pacejka_car_input& input)
{
  Eigen::Matrix<double, 4, 1> measured_state;  // The four angles that one lighthouse sweep measures
  measurement_function(commons::convertToConstVector(state), commons::convertToVector<4, 1>(measured_state));
  return measured_state;
}

/**
 * @brief Get the Numerical Jacobian for a given state
 *
 * @param state current state
 * @param H the state jacobian dh/dx
 */
void LighthouseSensorModel::getNumericalJacobian(const crs_models::pacejka_model::pacejka_car_state& state,
                                            const crs_models::pacejka_model::pacejka_car_input& input,
                                            Eigen::Matrix<double, -1, 6>& H)
{
  casadi::Function jacobian_fn = getSymbolicJacobian();

  // Prepare inputs for jacobian function

  // the jacobian function expects 6 + 2 + 6 arguments, since it can deal with implicit representation.
  // Our function is: f(x,y,yaw,vx,vy,dyaw,u0,u1) -> (x_d,y_d,yaw_d,vx_d,vy_d,dyaw_d)
  // jacobian_fn() returns:
  // f(x,y,yaw,vx,vy,dyaw,u0,u1, x_d,y_d,yaw_d,vx_d,vy_d,dyaw_d) -> (dx_d/dx, dx_d/dy, dx_d/dyaw ...)
  // Lets just add zeros at the end since we don't have any implicit dependencies

  auto state_and_implicit = commons::convertToConstVector(state);

  crs_models::pacejka_model::pacejka_car_state unused_implicit_inputs;
  auto unused_implicit_inputs_vec = commons::convertToConstVector(unused_implicit_inputs);
  state_and_implicit.insert(state_and_implicit.end(), unused_implicit_inputs_vec.begin(),
                            unused_implicit_inputs_vec.end());  // Adds unused_implicit_inputs at end of  fnc_input
  // Call jacobian
  jacobian_fn(state_and_implicit, commons::convertToVector<-1, 6>(H));
}

/**
 * @brief Get the Algebraic Jacobian as casadi function.
 *
 * @return casadi::Function
 */
casadi::Function LighthouseSensorModel::getSymbolicJacobian()
{
  return measurement_function.jac();
}

}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models