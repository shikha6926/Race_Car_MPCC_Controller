#ifndef PACEJKA_MPC_SAFETY_FILTER_CONFIG_H
#define PACEJKA_MPC_SAFETY_FILTER_CONFIG_H

#include <safety_framework/mpc_based_safety_filter.h>

#include <mpc_solvers/pacejka_safety_solver.h>

#include <pacejka_model/pacejka_discrete.h>
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_params.h>

#include <commons/trajectory.h>
#include <commons/static_track_trajectory.h>

namespace crs_safety
{
struct pacejka_mpc_safety_config
{
  double dist_targ_multiplier;
  double min_terminal_dist;
  double max_terminal_dist;

  bool use_torque_filter;
  double dist_decrement_max;

  std::string solver_type;

  double loookahead_time;

  std::string reference_method;

  double threshold;

  double cost_steer;
  double cost_torque;
  double cost_delta_torque;
  double cost_delta_steer;
};
}  // namespace crs_safety
#endif /* PACEJKA_MPC_SAFETY_FILTER_CONFIG_H */
