
#include "pacejka_sensor_model/imu_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{

// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
/**
 * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the pacejka state and therefore
 * the continuous model is needed
 *
 * @param pacejka_cont the continuous model
 * @param R measurement covariance Matrix
 */
ImuSensorModel::ImuSensorModel(const std::shared_ptr<crs_models::pacejka_model::ContinuousPacejkaModel> pacejka_cont,
                               const Eigen::Matrix3d& R)
  : SensorModel(3, ImuSensorModel::SENSOR_KEY)  // Measurement dimension is three
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),   casadi::MX::sym("yaw"),
                                       casadi::MX::sym("v_x"), casadi::MX::sym("v_y"), casadi::MX::sym("yaw_rate") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("steer"), casadi::MX::sym("torque") };

  auto cont_dynamics = pacejka_cont->getContinuousDynamics(state_mx, input_mx);  // returns x_dot, y_dot, theta_dot,
                                                                                 // vx_dot, vy_dot, yaw_dot_dot
  //                                                 vx_dot,           vy_dot,        theta_dot,  
  std::vector<casadi::MX> measured_states_mx = { cont_dynamics[3], cont_dynamics[4], cont_dynamics[2] };

  state_mx.insert(state_mx.end(), input_mx.begin(), input_mx.end());  // Append input at the end of state vector
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);
  R_ = R;
}

const std::string ImuSensorModel::SENSOR_KEY = "imu";

/**
 * @brief Evaluates the measurement model at the given state
 *
 * @param state
 * @return double vector of measured states (acc_x,acc_y,d_yaw)
 */
Eigen::Matrix<double, -1, 1> ImuSensorModel::applyModel(const crs_models::pacejka_model::pacejka_car_state& state,
                                                        const crs_models::pacejka_model::pacejka_car_input& input)
{
  Eigen::Matrix<double, 3, 1> measured_state;  // x, y, yaw = what Imu can measure
  measurement_function(commons::convertToConstVector(state, input), commons::convertToVector<3, 1>(measured_state));
  return measured_state;
}

/**
 * @brief Get the Numerical Jacobian for a given state
 *
 * @param state current state
 * @param H the state jacobian dh/dx
 */
void ImuSensorModel::getNumericalJacobian(const crs_models::pacejka_model::pacejka_car_state& state,
                                          const crs_models::pacejka_model::pacejka_car_input& input,
                                          Eigen::Matrix<double, -1, 6>& H)
{
  casadi::Function jacobian_fn = getSymbolicJacobian();

  // Prepare inputs for jacobian function

  // the jacobian function expects 6 + 2 + 6 arguments, since it can deal with implicit representation.
  // Lets just add zeros at the end since we don't have any implicit dependencies

  auto state_and_implicit = commons::convertToConstVector(state, input);

  crs_models::pacejka_model::pacejka_car_state unused_implicit_inputs;
  auto unused_implicit_inputs_vec = commons::convertToConstVector(unused_implicit_inputs);
  state_and_implicit.insert(state_and_implicit.end(), unused_implicit_inputs_vec.begin(),
                            unused_implicit_inputs_vec.end());  // Adds unused_implicit_inputs at end of  fnc_input
  // Call jacobian (3 = nr of measurments, 14 = 2*states + inputs)
  Eigen::Matrix<double, 3, 14> full_jacobian = Eigen::Matrix<double, 3,14>::Zero();  
  jacobian_fn(state_and_implicit,commons::convertToVector<3, 14>(full_jacobian));
  H = full_jacobian.block(0, 0, 3, 6);               // df/dx
}

/**
 * @brief Get the Algebraic Jacobian as casadi function.
 *
 * @return casadi::Function
 */
casadi::Function ImuSensorModel::getSymbolicJacobian()
{
  return measurement_function.jac();
}

}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models