
#include "pacejka_sensor_model/vicon_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{

// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
ViconSensorModel::ViconSensorModel(const Eigen::Matrix3d& R)
  : SensorModel(3, ViconSensorModel::SENSOR_KEY)  // Measurement dimension is three
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),   casadi::MX::sym("yaw"),
                                       casadi::MX::sym("v_x"), casadi::MX::sym("v_y"), casadi::MX::sym("yaw_rate") };
  std::vector<casadi::MX> measured_states_mx = { state_mx[0], state_mx[1], state_mx[2] };
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);
  R_ = R;
}

const std::string ViconSensorModel::SENSOR_KEY = "vicon";

/**
 * @brief Evaluates the measurement model at the given state
 *
 * @param state
 * @return double vector of measured states (x,y,yaw)
 */
Eigen::Matrix<double, -1, 1> ViconSensorModel::applyModel(const crs_models::pacejka_model::pacejka_car_state& state,
                                                          const crs_models::pacejka_model::pacejka_car_input& input)
{
  Eigen::Matrix<double, 3, 1> measured_state;  // x, y, yaw = what vicon can measure
  measurement_function(commons::convertToConstVector(state), commons::convertToVector<3, 1>(measured_state));
  return measured_state;
}

/**
 * @brief Get the Numerical Jacobian for a given state
 *
 * @param state current state
 * @param H the state jacobian dh/dx
 */
void ViconSensorModel::getNumericalJacobian(const crs_models::pacejka_model::pacejka_car_state& state,
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
casadi::Function ViconSensorModel::getSymbolicJacobian()
{
  return measurement_function.jac();
}

}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models