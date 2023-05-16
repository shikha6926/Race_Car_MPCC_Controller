
#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "custom_model_template/custom_model_continuous.h"
#include "custom_model_template/custom_model_discrete.h"
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <dynamic_models/utils/data_conversion.h>

namespace crs_models
{
namespace custom_model
{

DiscreteCustomModel::DiscreteCustomModel(custom_params params, Eigen::Matrix<double, 6, 6> Q)
  : DiscreteDynamicModelWrapper(Q)
{
  cont_model = std::make_unique<crs_models::custom_model::ContinuousCustomModel>(params);

  // Setup integrator
  using namespace casadi;  // somehow this is needed to use vertcat(), casadi::vertcat() does not work

  // ==================== TODO define your symbols here ====================
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),   casadi::MX::sym("yaw"),
                                       casadi::MX::sym("v_x"), casadi::MX::sym("v_y"), casadi::MX::sym("yaw_rate") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("torque"), casadi::MX::sym("steer"), casadi::MX::sym("Ts") };
  // ==================== END TODO ====================

  std::vector<casadi::MX> state_dot_mx = cont_model->getContinuousDynamics(state_mx, input_mx);

  auto x = vertcat(state_mx);
  auto p = vertcat(input_mx);
  auto ode = vertcat(state_dot_mx);

  casadi::MXDict dae = { // list of lists
                         { "x", x },
                         { "p", p },
                         { "ode", ode * input_mx[2] }
  };  // The main problem we want to solve
  casadi::Dict opts = { { "tf", 1 } };

  // Create integrator function, using runge kutta for now
  // TODO(sabodmer) create discretization_params struct with field "intergation_method" and pass it to model in
  // constructor to remove hardcoded rk
  integrator_ = casadi::integrator("cont_dynamics_integrator", "rk", dae, opts);
}

/**
 * @brief integrates the state for a given integration_time
 *
 * @param state
 * @param control_input
 * @param integration_time
 * @return custom_state returns the integrated state
 */
custom_state DiscreteCustomModel::applyModel(const custom_state state, const custom_input control_input,
                                             double integration_time)
{
  assert(integration_time >= 0);  // Model can not go backwards in time

  if (integration_time == 0)
  {
    return state;
  }

  // This solves the differential equation dx/dt from t = 0 to t = ts using x(0) = x0

  // ==================== TODO update argument mapping according to your struct definition ====================
  casadi::Function::MapArg arg = {
    { "x0", { state.pos_x, state.pos_y, state.yaw, state.vel_x, state.vel_y, state.yaw_rate } },
    { "p", { control_input.torque, control_input.steer, integration_time } }
  };
  custom_state output_state;
  // ==================== END TODO ====================

  auto vec = commons::convertToVector(output_state);
  // TODO(@zrene), this is needed since the casadi integrator returns "6" entries (first one being the state of
  // dimension 4, everything else gets discarded) This is an ugly fix to make sure that states smaller than 6 can be
  // passed to the integrator. Maybe use casadi::Function::MapRes or something similar to populate output
  while (vec.size() < integrator_.n_out())
    vec.push_back(nullptr);

  integrator_(integrator_.buf_in(arg), vec);

  return output_state;
}

}  // namespace custom_model

}  // namespace crs_models