
#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "pacejka_model/pacejka_car_input.h"
#include "pacejka_model/pacejka_car_state.h"
#include "pacejka_model/pacejka_continuous.h"
#include "pacejka_model/pacejka_discrete.h"
#include "pacejka_model/pacejka_params.h"
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <dynamic_models/utils/data_conversion.h>

namespace crs_models
{
namespace pacejka_model
{

typedef Eigen::Matrix<double, 6, 6> StateMatrix;
typedef Eigen::Matrix<double, 6, 2> InputMatrix;
// Option 1: Create an object and specify the process noise covariance matrix Q yourself.
DiscretePacejkaModel::DiscretePacejkaModel(pacejka_params params, Eigen::Matrix<double, 6, 6> Q,
                                           std::string integration_method /* rk (default) */)
  : params_(params), DiscreteDynamicModelWrapper(Q)
{
  cont_model = std::make_unique<crs_models::pacejka_model::ContinuousPacejkaModel>(params);

  // Setup integrator
  using namespace casadi;  // somehow this is needed to use vertcat(), casadi::vertcat() does not work
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"),   casadi::MX::sym("y"),   casadi::MX::sym("yaw"),
                                       casadi::MX::sym("v_x"), casadi::MX::sym("v_y"), casadi::MX::sym("yaw_rate") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("torque"), casadi::MX::sym("steer"), casadi::MX::sym("Ts") };
  std::vector<casadi::MX> state_dot_mx = cont_model->getContinuousDynamics(state_mx, { input_mx[0], input_mx[1] });

  auto x = vertcat(state_mx);
  auto p = vertcat(input_mx);
  auto ode = vertcat(state_dot_mx);

  casadi::MXDict dae = { // list of lists
                         { "x", x },
                         { "p", p },
                         { "ode", ode * input_mx[2] }
  };  // The main problem we want to solve
  casadi::Dict opts = { { "tf", 1 } };

  integrator_ = casadi::integrator("cont_dynamics_integrator", integration_method, dae, opts);
}

/**
 * @brief integrates the state for a given integration_time
 *
 * @param state
 * @param control_input
 * @param integration_time
 * @return pacejka_car_state returns the integrated state
 */
pacejka_car_state DiscretePacejkaModel::applyModel(const pacejka_car_state state, const pacejka_car_input control_input,
                                                   double integration_time)
{
  assert(integration_time >= 0);  // Model can not go backwards in time

  if (integration_time == 0)
  {
    return state;
  }

  // This solves the differential equation dx/dt from t = 0 to t = ts using x(0) = x0
  casadi::Function::MapArg arg = {
    { "x0", { state.pos_x, state.pos_y, state.yaw, state.vel_x, state.vel_y, state.yaw_rate } },
    { "p", { control_input.torque, control_input.steer, integration_time } }
  };
  pacejka_car_state output_state;

  auto vec = commons::convertToVector(output_state);
  // TODO(@zrene), this is needed since the casadi integrator returns "6" entries (first one being the state of
  // dimension 4, everything else gets discarded) This is an ugly fix to make sure that states smaller than 6 can be
  // passed to the integrator. Maybe use casadi::Function::MapRes or something similar to populate output
  while (vec.size() < integrator_.n_out())
    vec.push_back(nullptr);

  integrator_(integrator_.buf_in(arg), vec);

  return output_state;
}
}  // namespace pacejka_model
}  // namespace crs_models