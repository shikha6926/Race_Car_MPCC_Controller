#include <casadi/casadi.hpp>

#include "custom_model_template/custom_model_continuous.h"

namespace crs_models
{
namespace custom_model
{
ContinuousCustomModel::ContinuousCustomModel(custom_params params) : params(params)  // Constructor
{
  // <================= TODO define your model inputs and state in with variable names here =================>
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),   casadi::MX::sym("yaw"),
                                       casadi::MX::sym("v_x"), casadi::MX::sym("v_y"), casadi::MX::sym("yaw_rate") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("torque"), casadi::MX::sym("steer") };
  // <================= END REQUIRED CHANGES =================>

  std::vector<casadi::MX> state_dot_mx = getContinuousDynamics(state_mx, input_mx);
  // append control inputs to state vector (casadi functions only take one input)
  for (const auto input : input_mx)
    state_mx.push_back(input);

  dynamics_f_ = casadi::Function("f_dot", state_mx, state_dot_mx);

  jacobian_fn_ = getSymbolicJacobian();
}  // Constructor

/**
 * @brief evaluated state_dot (= f(x,u)) at current state and control input
 *
 * @param state
 * @param input
 * @return custom_state returns state_dot evaluated at current state and control input
 */
custom_state ContinuousCustomModel::applyModel(const custom_state state, const custom_input input)
{
  custom_state state_dot_numerical;
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
void ContinuousCustomModel::getNumericalJacobian(const custom_state& state, const custom_input& control_input,
                                                 StateMatrix& A, InputMatrix& B)
{
  auto state_and_input = commons::convertToConstVector(state, control_input);

  // Prepare inputs for jacobian function
  auto& state_and_input_and_implicit = state_and_input;
  custom_state unused_implicit_inputs;  // these here are only used to evaluate the jocbian if there are any implicit
                                        // dependencies.
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
std::vector<casadi::MX> ContinuousCustomModel::getContinuousDynamics(const std::vector<casadi::MX> state,
                                                                     const std::vector<casadi::MX> control_input)
{
  // ========================= TODO IMPLEMENT YOUR DYNAMICS HERE =========================
  using namespace casadi;

  // Just for better readability, function inputs (states and control inputs)
  auto x = state[0];  // Make sure to match these with states MX defined in constructor
  auto y = state[1];
  auto yaw = state[2];
  auto v_x = state[3];
  auto v_y = state[4];
  auto yaw_rate = state[5];
  auto torque = control_input[0];
  auto steer = control_input[1];

  // TODO @(sabodmer) this is not so nice. Somehow casadi::sin() does not work but using namespace casadi, sin() works?
  // Intermediate function values
  auto Fx = (params.Cm1 - params.Cm2 * v_x) * torque - params.Cd * v_x * v_x - params.Croll;
  auto beta = atan2(v_y, v_x);
  auto ar = atan2(-v_y + params.lr * yaw_rate, v_x);
  auto af = steer + atan2(-v_y - params.lf * yaw_rate, v_x);
  auto Fr = params.Dr * sin(params.Cr * atan(params.Br * ar));
  auto Ff = params.Df * sin(params.Cf * atan(params.Bf * af));

  // Output of f(state, control)
  auto x_dot = v_x * cos(yaw) - v_y * sin(yaw);
  auto y_dot = v_x * sin(yaw) + v_y * cos(yaw);
  auto yaw_dot = yaw_rate;
  auto v_x_dot = 1.0 / params.m * (Fx - Ff * sin(steer) + params.m * v_y * yaw_rate);
  auto v_y_dot = 1.0 / params.m * (Fr + Ff * cos(steer) - params.m * v_x * yaw_rate);
  auto yaw_rate_dot = 1.0 / params.I * (Ff * params.lf * cos(steer) - Fr * params.lr);

  auto state_dot = { x_dot, y_dot, yaw_dot, v_x_dot, v_y_dot, yaw_rate_dot };
  return state_dot;
  // ========================= END TODO =========================
}

}  // namespace custom_model
}  // namespace crs_models