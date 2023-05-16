
#include "pacejka_mpc_safety_filter/pacejka_mpc_safety_filter.h"
#include "acados_pacejka_safety_model_solver/acados_pacjeka_safety_model_solver.h"
#include <chrono>
#include <unistd.h>

namespace crs_safety
{

PacejkaMpcSafetyFilter::PacejkaMpcSafetyFilter(pacejka_mpc_safety_config config,
                                               std::shared_ptr<crs_controls::StaticTrackTrajectory> trajectory,
                                               std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model)
  : MpcBasedSafetyFilter(config.solver_type, model), config_(config)
{
  /* Make sure to initialize solver here!! */
  solver = getSolver(config_.solver_type);
  trajectory_ = trajectory;

  // Fill reference with zeros
  for (int i = 0; i < solver->getHorizonLength(); i++)
  {
    planned_trajectory.push_back({ 0, 0, 0, 0, 0, 0, 0 });
  }
};

std::vector<std::pair<int, crs_models::pacejka_model::pacejka_car_state>>
PacejkaMpcSafetyFilter::calculateReferenceTrajectory(const crs_models::pacejka_model::pacejka_car_state state,
                                                     const crs_models::pacejka_model::pacejka_car_input input)
{
  std::vector<std::pair<int, crs_models::pacejka_model::pacejka_car_state>> ref_trajectory;
  int ref_idx = trajectory_->getClosestTrackPointIdx(Eigen::Vector2d({ state.pos_x, state.pos_y }));
  double arc_length_delta = trajectory_->getArcLength(1) - trajectory_->getArcLength(0);

  // Calculate distance too look ahead and select
  // reference points with equal distance on track
  if (config_.reference_method == "uniform")
  {
    double time_horizon_ = float(solver->getHorizonLength()) * solver->getSamplePeriod();
    // Get average distance between two points on track
    // double dist_target = time_horizon_ * input.torque * config_.dist_targ_multiplier;
    double dist_target =
        time_horizon_ * input.torque * config_.dist_targ_multiplier + 0.5 * state.vel_x * time_horizon_;

    if (config_.use_torque_filter)
    {
      // filter torque down when desired torque drops
      if (prev_dist_target_ > dist_target)
      {
        dist_target = std::max(prev_dist_target_ - config_.dist_decrement_max, dist_target);
      }
      prev_dist_target_ = dist_target;
    }

    // compute terminal idx based on the distance target
    double terminal_dist = std::min(config_.max_terminal_dist, std::max(config_.min_terminal_dist, dist_target));
    // Convert distance value to index
    double terminal_idx_dist = std::floor(terminal_dist / arc_length_delta);
    int constraint_idx_delta = std::max(1, (int)std::floor(terminal_idx_dist / solver->getHorizonLength()));

    for (int stage = 0; stage < solver->getHorizonLength(); stage++)
    {
      auto ref_pt = trajectory_->operator[](ref_idx + stage * constraint_idx_delta);
      crs_models::pacejka_model::pacejka_car_state reference;
      reference.pos_x = ref_pt.x();
      reference.pos_y = ref_pt.y();
      reference.yaw = trajectory_->getTrackAngle(ref_idx + stage * constraint_idx_delta);

      // Warm start velocity
      reference.vel_x = state.vel_x + (1 - state.vel_x) / solver->getHorizonLength() * (stage);

      reference.vel_y = 0;
      reference.yaw_rate = 0;
      ref_trajectory.push_back(std::make_pair(ref_idx + stage * constraint_idx_delta, reference));
    }
  }
  // Propagate car state forward using constant inputs and use these as refrence points
  else if (config_.reference_method == "const_input")
  {
    crs_models::pacejka_model::pacejka_car_state start_pt;
    start_pt.vel_x = state.vel_x;

    for (int stage = 0; stage < solver->getHorizonLength(); stage++)
    {
      start_pt = model->applyModel(start_pt, input, solver->getSamplePeriod());
      int idx_delta = std::ceil(start_pt.pos_x / arc_length_delta);
      auto ref_pt = trajectory_->operator[](ref_idx + idx_delta);

      crs_models::pacejka_model::pacejka_car_state reference = {
        ref_pt.x(), ref_pt.y(), trajectory_->getTrackAngle(ref_idx + idx_delta), start_pt.vel_x, 0, 0
      };
      ref_trajectory.push_back(std::make_pair(ref_idx + idx_delta, reference));
    }
  }
  else
  {
    std::cout << "[WARN] unknown reference method: " << config_.reference_method << std::endl;
  }
  return ref_trajectory;
}  // namespace crs_safety
/**
 * @brief Wraps an angle to [-pi, pi]
 *
 * @param angle the input angle
 * @return double the wrapped angle
 */
double wrapToPi(double angle)
{
  double x = std::fmod(angle + M_PI, 2 * M_PI);
  if (x < 0)
    x += 2 * M_PI;
  return x - M_PI;
}

crs_models::pacejka_model::pacejka_car_input
PacejkaMpcSafetyFilter::getSafeControlInput(const crs_models::pacejka_model::pacejka_car_state state,
                                            const crs_models::pacejka_model::pacejka_car_input control_input)
{
  auto future_state = model->applyModel(state, control_input, config_.loookahead_time);

  // set track and terminal constraints
  int closest_track_idx =
      trajectory_->getClosestTrackPointIdx(Eigen::Vector2d({ future_state.pos_x, future_state.pos_y }));

  // Fix yaw from state estimate (it is off by multiple of 2*pi)
  float track_yaw = trajectory_->getTrackAngle(closest_track_idx);
  future_state.yaw = track_yaw + wrapToPi(future_state.yaw - track_yaw);

  double x_init[solver->getStateDimension()];
  x_init[0] = future_state.pos_x;
  x_init[1] = future_state.pos_y;
  x_init[2] = future_state.yaw;
  x_init[3] = std::max(0.01, future_state.vel_x);  // Should not be zero as otherwise the model gets NaN issues
  x_init[4] = future_state.vel_y;
  x_init[5] = future_state.yaw_rate;
  x_init[6] = control_input.torque;
  x_init[7] = control_input.steer;

  solver->setInitialState(x_init);

  std::vector<std::pair<int, crs_models::pacejka_model::pacejka_car_state>> references =
      calculateReferenceTrajectory(future_state, control_input);

  auto terminal_reference = references[references.size() - 1].second;

  // Run solver
  for (int current_stage = 0; current_stage < solver->getHorizonLength(); current_stage++)
  {
    auto reference_state = references[current_stage].second;

    // Update tracking point based on predicted distance on track
    mpc_solvers::pacejka_safety_solvers::reference_on_track reference = {
      reference_state.pos_x,    reference_state.pos_y,    reference_state.yaw,
      terminal_reference.pos_x, terminal_reference.pos_y, terminal_reference.yaw,
    };

    solver->updateParams(current_stage,
                         model->getParams(),  // Model Dynamics
                         { control_input.torque, control_input.steer, config_.cost_torque, config_.cost_steer,
                           config_.cost_delta_torque, config_.cost_delta_steer },  // Input
                         reference                                                 // Tracking point
    );

    // Only used for visualization
    if (current_stage != solver->getHorizonLength() - 1)
    {
      planned_trajectory[current_stage][4] = reference.xp_track;
      planned_trajectory[current_stage][5] = reference.yp_track;
      planned_trajectory[current_stage][6] = reference.yaw_track;
    }
    else
    {
      planned_trajectory[current_stage][4] = reference.xp_e;
      planned_trajectory[current_stage][5] = reference.yp_e;
      planned_trajectory[current_stage][6] = reference.yaw_track;
    }

    // warm start the solver with dummy trajectory
    double u_l_t_warm = 0.3;
    double delta_ff =
        (trajectory_->getCurvature(references[current_stage].first)) * (model->getParams().lf + model->getParams().lr);

    double torque_decay = 0.0;
    double u = u_l_t_warm * std::exp(-torque_decay * current_stage);
    double x_i[8] = { reference_state.pos_x,
                      reference_state.pos_y,
                      reference_state.yaw,
                      reference_state.vel_x,
                      reference_state.vel_y,
                      reference_state.yaw_rate,
                      u,
                      delta_ff };

    solver->setStateInitialGuess(current_stage, x_i);
  }

  double x[solver->getHorizonLength() * solver->getStateDimension()];
  double u[solver->getHorizonLength() * solver->getInputDimension()];

  if (solver->solve(x, u))
  {
    return control_input;
  }

  double solve_time =
      std::dynamic_pointer_cast<mpc_solvers::pacejka_safety_solvers::AcadosPacejkaSafetySolver>(solver)->getSolveTime();
  if (solve_time > 1 / solver->getSamplePeriod())
  {
    std::cout << "[WARNING] Solve time exceeded sample time: " << solve_time << "(solve time) "
              << 1 / solver->getSamplePeriod() << "(period)" << std::endl;
  }
  for (int i = 0; i < solver->getHorizonLength(); i++)
  {
    planned_trajectory[i][0] = x[i * solver->getStateDimension() + mpc_solvers::pacejka_safety_solvers::vars::X];
    planned_trajectory[i][1] = x[i * solver->getStateDimension() + mpc_solvers::pacejka_safety_solvers::vars::Y];
    planned_trajectory[i][2] = x[i * solver->getStateDimension() + mpc_solvers::pacejka_safety_solvers::vars::VX];
    planned_trajectory[i][3] = x[i * solver->getStateDimension() + mpc_solvers::pacejka_safety_solvers::vars::YAW];
  }

  // Modify input
  crs_models::pacejka_model::pacejka_car_input safe_input;
  safe_input.steer = x[1 * solver->getStateDimension() + mpc_solvers::pacejka_safety_solvers::vars::STEER];
  safe_input.torque = x[1 * solver->getStateDimension() + mpc_solvers::pacejka_safety_solvers::vars::TORQUE];

  input_overridden =
      Eigen::Vector2d(safe_input.steer - control_input.steer, safe_input.torque - control_input.torque).norm() >
      config_.threshold;

  return safe_input;
};

std::shared_ptr<mpc_solvers::MpcSolver<crs_models::pacejka_model::pacejka_params,
                                       mpc_solvers::pacejka_safety_solvers::reference_input,
                                       mpc_solvers::pacejka_safety_solvers::reference_on_track>>

PacejkaMpcSafetyFilter::getSolver(std::string solver_type)
{
  if (solver_type == "acados")
  {
    return std::static_pointer_cast<mpc_solvers::MpcSolver<crs_models::pacejka_model::pacejka_params,
                                                           mpc_solvers::pacejka_safety_solvers::reference_input,
                                                           mpc_solvers::pacejka_safety_solvers::reference_on_track>>(
        std::make_shared<mpc_solvers::pacejka_safety_solvers::AcadosPacejkaSafetySolver>());
  }

  return nullptr;
};

};  // namespace crs_safety