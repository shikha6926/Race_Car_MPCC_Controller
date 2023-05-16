
#include "kinematic_sensor_model/imu_sensor_model.h"
#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
namespace kinematic_sensor_models
{

// Option 2: Create an object and specify the process noise covariance matrix Q yourself.
/**
 * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the kinematic state and
 * therefore the continuous model is needed
 *
 * @param kinematic_cont the continuous model
 * @param R measurement covariance Matrix
 */
ImuSensorModel::ImuSensorModel(
    const std::shared_ptr<crs_models::kinematic_model::ContinuousKinematicModel> kinematic_cont,
    const Eigen::Matrix3d& R)
  : SensorModel(3, ImuSensorModel::SENSOR_KEY)  // Measurement dimension is three
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"), casadi::MX::sym("y"), casadi::MX::sym("yaw"),
                                       casadi::MX::sym("v") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("steer"), casadi::MX::sym("torque") };

  auto cont_dynamics = kinematic_cont->getContinuousDynamics(state_mx, input_mx);  // returns x_dot, y_dot, theta_dot,
                                                                                   // vx_dot, vy_dot, yaw_dot_dot
  auto v_dot = cont_dynamics[3];
  auto tmp1 = tan(input_mx[0]) * kinematic_cont->getParams().lr;
  auto tmp2 = kinematic_cont->getParams().lf + kinematic_cont->getParams().lr;
  auto beta = atan2(tmp1, tmp2);
  //                                             theta_dot,        vx_dot,           vy_dot,
  std::vector<casadi::MX> measured_states_mx = { cont_dynamics[2], v_dot * cos(beta), v_dot * sin(beta) };
  state_mx.insert(state_mx.end(), input_mx.begin(), input_mx.end());  // Append input at the end of state vector
  measurement_function = casadi::Function("applyMeasurementModel", state_mx, measured_states_mx);
  R_ = R;
}

const std::string ImuSensorModel::SENSOR_KEY = "imu";

/**
 * @brief Evaluates the measurement model at the given state
 *
 * @param state
 * @return double vector of measured states (dvx,dyv,dyaw)
 */
Eigen::Matrix<double, -1, 1> ImuSensorModel::applyModel(const crs_models::kinematic_model::kinematic_car_state& state,
                                                        const crs_models::kinematic_model::kinematic_car_input& input)
{
  Eigen::Matrix<double, 3, 1> measured_state;  // dvx, dvy, dyaw = what Imu can measure
  measurement_function(commons::convertToConstVector(state, input), commons::convertToVector<3, 1>(measured_state));
  return measured_state;
}

/**
 * @brief Get the Numerical Jacobian for a given state
 *
 * @param state current state
 * @param H the state jacobian dh/dx
 */
void ImuSensorModel::getNumericalJacobian(const crs_models::kinematic_model::kinematic_car_state& state,
                                          const crs_models::kinematic_model::kinematic_car_input& input,
                                          Eigen::Matrix<double, -1, 4>& H)
{
  casadi::Function jacobian_fn = getSymbolicJacobian();

  // Prepare inputs for jacobian function

  // the jacobian function expects 6 + 2 + 6 arguments, since it can deal with implicit representation.
  // Our function is: f(x,y,yaw,vx,vy,dyaw,u0,u1) -> (x_d,y_d,yaw_d,vx_d,vy_d,dyaw_d)
  // jacobian_fn() returns:
  // f(x,y,yaw,vx,vy,dyaw,u0,u1, x_d,y_d,yaw_d,vx_d,vy_d,dyaw_d) -> (dx_d/dx, dx_d/dy, dx_d/dyaw ...)
  // Lets just add zeros at the end since we don't have any implicit dependencies

  auto state_and_implicit = commons::convertToConstVector(state, input);

  crs_models::kinematic_model::kinematic_car_state unused_implicit_inputs;
  auto unused_implicit_inputs_vec = commons::convertToConstVector(unused_implicit_inputs);
  state_and_implicit.insert(state_and_implicit.end(), unused_implicit_inputs_vec.begin(),
                            unused_implicit_inputs_vec.end());  // Adds unused_implicit_inputs at end of  fnc_input
  // Call jacobian
  jacobian_fn(state_and_implicit, commons::convertToVector<-1, 4>(H));
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

}  // namespace kinematic_sensor_models
}  // namespace crs_sensor_models