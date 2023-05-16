#include <dynamic_models/utils/data_conversion.h>
#include "kinematic_model/kinematic_continuous.h"
#include <casadi/casadi.hpp>

namespace crs_models
{
namespace kinematic_model
{
ContinuousKinematicModel::ContinuousKinematicModel(kinematic_params params) : params(params)
{
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"), casadi::MX::sym("y"), casadi::MX::sym("yaw"),
                                       casadi::MX::sym("velocity") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("torque"), casadi::MX::sym("steer") };

  std::vector<casadi::MX> state_dot_mx = getContinuousDynamics(state_mx, input_mx);
  // append control inputs to state vector (casadi functions only take one input)
  state_mx.push_back(input_mx[0]);
  state_mx.push_back(input_mx[1]);
  dynamics_f_ = casadi::Function("f_kinematic_cont", state_mx, state_dot_mx);
}  // Constructor

/**
 * @brief evaluated state_dot (= f(x,u)) at current state and control input
 *
 * @param state
 * @param input
 * @return kinematic_car_state returns state_dot evaluated at current state and control input
 */
kinematic_car_state ContinuousKinematicModel::applyModel(const kinematic_car_state state,
                                                         const kinematic_model::kinematic_car_input input)
{
  kinematic_car_state state_dot_numerical;
  // Convert casadi symbols (mx) to casadi function to be evaluated at current state & control input
  dynamics_f_(commons::convertToConstVector(state, input), commons::convertToVector(state_dot_numerical));
  return state_dot_numerical;
};

/**
 * @brief Evaluate the Symbolic Jacobian at a given state and control input. The Jacobians are saved in A, B
 *
 * @param state
 * @param control_input
 * @param A empty matrix to fill as Jacobian df/dx
 * @param B empty matrix to fill as Jacobian df/du
 */
void ContinuousKinematicModel::getNumericalJacobian(const kinematic_car_state& state,
                                                    const kinematic_model::kinematic_car_input& control_input,
                                                    StateMatrix& A, InputMatrix& B)
{
  // Preprocessing state + inputs.
  // The Jacobian function expects (State + Input + State) as one input vector
  // Note that the last State part is only used if there are implicit function definitions and can usely be empty.

  auto state_and_input = commons::convertToConstVector(state, control_input);
  // Prepare inputs for jacobian function
  auto& state_and_input_and_implicit = state_and_input;
  kinematic_car_state unused_implicit_inputs;
  auto unused_implicit_inputs_vec = commons::convertToConstVector(unused_implicit_inputs);
  state_and_input_and_implicit.insert(state_and_input.end(), unused_implicit_inputs_vec.begin(),
                                      unused_implicit_inputs_vec.end());  // Adds unused_implicit_inputs at end of
                                                                          // fnc_input

  getNumericalJacobianInternal(state_and_input_and_implicit, A, B);
}

/**
 * @brief Get the analytical state equations f(state, input) = state_dot
 *
 * @param state
 * @param control_input
 * @return std::vector<casadi::MX> returns the analytical state equations f(state, input) = state_dot
 */
std::vector<casadi::MX> ContinuousKinematicModel::getContinuousDynamics(const std::vector<casadi::MX> state,
                                                                        const std::vector<casadi::MX> control_input)
{
  using namespace casadi;
  assert(state.size() == 4);
  assert(control_input.size() == 2);

  // Just for better readability, function inputs (states and control inputs)
  auto x = state[0];
  auto y = state[1];
  auto yaw = state[2];
  auto v = state[3];
  auto torque = params.a * control_input[0] + params.b;
  auto steer = control_input[1];

  auto beta = atan2(tan(steer) * params.lr, params.lf + params.lr);

  // Output of f(state, control)
  auto x_dot = v * cos(yaw + beta);
  auto y_dot = v * sin(yaw + beta);
  auto v_dot = -1 / params.tau * v + 1 / params.tau * torque;
  auto yaw_dot = v * sin(beta) / params.lr;

  auto state_dot = { x_dot, y_dot, yaw_dot, v_dot };
  return state_dot;
}

}  // namespace kinematic_model
}  // namespace crs_models