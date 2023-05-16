#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include <dynamic_models/continuous_dynamic_model.h>
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <dynamic_models/utils/data_conversion.h>

#include "kinematic_model/kinematic_car_state.h"
#include "kinematic_model/kinematic_continuous.h"
#include "kinematic_model/kinematic_discrete.h"
#include "kinematic_model/kinematic_params.h"

namespace crs_models
{
namespace kinematic_model
{

// Option 1: Create an object and specify the process noise covariance matrix Q yourself.
DiscreteKinematicModel::DiscreteKinematicModel(kinematic_params params, Eigen::Matrix<double, 4, 4> Q,
                                               std::string integration_method /* rk4 (default) */)
  : DiscreteDynamicModelWrapper<kinematic_car_state, kinematic_model::kinematic_car_input, 4, 2>(Q)
{
  cont_model = std::make_unique<crs_models::kinematic_model::ContinuousKinematicModel>(params);

  // Setup integrator
  using namespace casadi;  // somehow this is needed to use vertcat(), casadi::vertcat() does not work
  std::vector<casadi::MX> state_mx = { casadi::MX::sym("x"), casadi::MX::sym("y"), casadi::MX::sym("yaw"),
                                       casadi::MX::sym("velocity") };
  std::vector<casadi::MX> input_mx = { casadi::MX::sym("torque"), casadi::MX::sym("steer"), casadi::MX::sym("Ts") };
  std::vector<casadi::MX> state_dot_mx = cont_model->getContinuousDynamics(state_mx, { input_mx[0], input_mx[1] });

  auto x = vertcat(state_mx);
  auto p = vertcat(input_mx);
  auto ode = vertcat(state_dot_mx);

  casadi::MXDict dae = { // list of lists
                         { "x", x },
                         { "p", p },
                         { "ode", ode * input_mx[2] }
  };  // The main problem we want to solve. input_mx[2] is integration time
  casadi::Dict opts = { { "tf", 1 } };

  integrator_ = casadi::integrator("cont_dynamics_integrator", integration_method, dae, opts);
}
/**
 * @brief integrates the state for a given integration_time
 *
 * @param state
 * @param control_input
 * @param integration_time
 * @return kinematic_car_state returns the integrated state
 */
kinematic_car_state DiscreteKinematicModel::applyModel(const kinematic_car_state state,
                                                       const kinematic_model::kinematic_car_input control_input,
                                                       double integration_time)
{
  assert(integration_time >= 0);  // Model can not go backwards in time
  if (integration_time == 0)
  {
    return state;
  }

  // This solves the differential equation dx/dt from t = 0 to t = ts using x(0) = x0
  casadi::Function::MapArg arg = { { "x0", { state.pos_x, state.pos_y, state.yaw, state.velocity } },
                                   { "p", { control_input.torque, control_input.steer, integration_time } } };

  kinematic_car_state output_state;

  auto vec = commons::convertToVector(output_state);
  // TODO(@zrene), this is needed since the casadi integrator returns "6" entries (first one being the state of
  // dimension 4, everything else gets discarded) This is an ugly fix to make sure that states smaller than 6 can be
  // passed to the integrator. Maybe use casadi::Function::MapRes or something similar to populate output
  while (vec.size() < integrator_.n_out())
    vec.push_back(nullptr);

  integrator_(integrator_.buf_in(arg), vec);

  return output_state;
}
}  // namespace kinematic_model
}  // namespace crs_models