#include <mpc_controller/pacejka_controller/tracking_mpc_pacejka_controller.h>

#ifdef acados_pacejka_tracking_mpc_solver_FOUND
#include "acados_pacejka_tracking_mpc_solver/acados_pacejka_tracking_mpc_solver.h"
#endif

namespace crs_controls
{

void PacejkaTrackingMpcController::loadMpcSolver()
{
  if (config_.solver_type == "ACADOS")
  {
#ifdef acados_pacejka_tracking_mpc_solver_FOUND
    solver_ = std::make_shared<mpc_solvers::pacejka_tracking_solvers::AcadosPacejkaTrackingMpcSolver>();
#else
    assert(true && "Requested ACADOS solver for pacejka MPC. Solver not found!");
#endif
  }
}

/*std::shared_ptr<StaticTrackTrajectory> PacejkaTrackingMpcController::getStaticTrack() // this I need to change
{
  return std::static_pointer_cast<StaticTrackTrajectory>(track_);
}*/

// Constructor, Creates a MPC controller based on the config and a track description
PacejkaTrackingMpcController::PacejkaTrackingMpcController(tracking_mpc_pacejka_config config,
                                             std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
                                             std::shared_ptr<Trajectory> track)
  : MpcController(model, std::static_pointer_cast<Trajectory>(track))
{
  setConfig(config);
  loadMpcSolver();

  // Initialize last solutions with zeros
  last_solution.states_ = std::vector<double>(solver_->getStateDimension() * solver_->getHorizonLength(), 0.0);
  last_solution.inputs_ = std::vector<double>(solver_->getInputDimension() * solver_->getHorizonLength(), 0.0);

  // Initialize last references with zeros (only for visualization)
  for (int i = 0; i < solver_->getHorizonLength(); i++)
    last_solution.reference_.push_back(Eigen::Vector2d(0, 0));
}

std::vector<std::vector<double>> PacejkaTrackingMpcController::getPlannedTrajectory()
{
  std::vector<std::vector<double>> traj;
  for (int i = 0; i < solver_->getHorizonLength(); i++)
  {
    double x_pos = last_solution.states_[i * solver_->getStateDimension() + pacejka_tracking_vars::X];
    double y_pos = last_solution.states_[i * solver_->getStateDimension() + pacejka_tracking_vars::Y];
    double vel = std::sqrt(std::pow(last_solution.states_[i * solver_->getStateDimension() + pacejka_tracking_vars::VX], 2) +
                           std::pow(last_solution.states_[i * solver_->getStateDimension() + pacejka_tracking_vars::VY], 2));
    // Append values to trajectory for visualization
    traj.push_back({
        x_pos,                                                                        // Planned x
        y_pos,                                                                        // Planned y
        vel,                                                                          // Planned velocity
        last_solution.states_[i * solver_->getStateDimension() + pacejka_tracking_vars::YAW],  // Planned Yaw
    });
  }

  return traj;
}

void PacejkaTrackingMpcController::initialize(crs_models::pacejka_model::pacejka_car_state state)
{
  int N_ = solver_->getHorizonLength();
  auto model_dynamics = model_->getParams();
  int N_WARM_START = 10;

  mpc_solvers::pacejka_tracking_solvers::tracking_costs mpc_costs = { config_.Q1, config_.Q2, config_.R1,
                                                             config_.R2 };

  // Initial inputs.
  last_input_.steer = 0;
  // Initial torque input.
  last_input_.torque = 0;

  double x_init[8];
  x_init[0] = state.pos_x;
  x_init[1] = state.pos_y;
  x_init[2] = state.yaw;
  x_init[3] = std::max(0.2, state.vel_x);  // Should not be zero as otherwise the model gets NaN issues
  x_init[4] = state.vel_y;
  x_init[5] = state.yaw_rate;
  x_init[6] = last_input_.torque;
  x_init[7] = last_input_.steer;

  double u_init[2];
  u_init[0] = 0.0;
  u_init[1] = 0.5;

  solver_->setInitialState(x_init);

  double u0[solver_->getInputDimension()] = { 0.0, last_input_.torque };

  // Setup parameter for initial solve
  for (int stage = 0; stage < solver_->getHorizonLength(); stage++)
  {
    // Set initial state
    for (int x_idx = 0; x_idx < solver_->getStateDimension(); x_idx++)
      last_solution.states_[stage * solver_->getStateDimension() + x_idx] = x_init[x_idx];

    // Set initial input (nothing to do, its zero)
    solver_->setStateInitialGuess(stage, x_init);
    solver_->setInputInitialGuess(stage, u_init);
  }

  // Run solver
  for (int run = 0; run < N_WARM_START; run++)
  {
    for (int stage = 0; stage < solver_->getHorizonLength(); stage++)
    {
      // Update tracking point
      mpc_solvers::pacejka_tracking_solvers::trajectory_track_point track_point;
      auto current_state = Eigen::Vector2d(state.pos_x, state.pos_y);
      auto next_track_point = trajectory_->getClosestTrackPoint(current_state);
      track_point.x = next_track_point(0);
      track_point.y = next_track_point(1);
      track_point.x = 0.5;
      track_point.y = 0.5;

      solver_->updateParams(stage,
                            model_dynamics,  // Model Dynamics
                            mpc_costs,       // Costs
                            track_point);    // Reference point

      // Use previous solution as initial guess this time
      solver_->setStateInitialGuess(stage, &last_solution.states_[stage * solver_->getStateDimension()]);
      solver_->setInputInitialGuess(stage, &last_solution.inputs_[stage * solver_->getInputDimension()]);
    }
    solver_->solve(&last_solution.states_[0], &last_solution.inputs_[0]);
  }
}

/**
 * @brief Executes MPC controller, returning a new input to apply
 *
 * @param state current measured state of the system
 * @param timestamp current timestamp, will be ignored
 * @return crs_models::pacejka_model::pacejka_car_input
 */
crs_models::pacejka_model::pacejka_car_input PacejkaTrackingMpcController::getControlInput(
    crs_models::pacejka_model::pacejka_car_state state, double timestamp /* timestamp will be ignored */)
{
  int N_ = solver_->getHorizonLength();

  mpc_solvers::pacejka_tracking_solvers::tracking_costs mpc_costs = { config_.Q1, config_.Q2, config_.R1,
                                                             config_.R2};

  if (!is_initialized)
  {
    initializing = true;
    initialize(state);
    initializing = false;
    is_initialized = true;
  }

  // initialize state to virtually advanced vehicle states and inputs
  //auto virtual_state = model_->applyModel(state, last_input_, config_.lag_compensation_time);

  double vel_x = std::max(0.2, state.vel_x);  // Should not be zero as otherwise the model gets NaN issues

  double x0[] = {
    state.pos_x, state.pos_y, state.yaw,
    vel_x, state.vel_y, state.yaw_rate,
    last_input_.torque,  last_input_.steer,
  };


  solver_->setInitialState(x0);
  // Run solver
  for (int current_stage = 0; current_stage < solver_->getHorizonLength(); current_stage++)
  {
    // next stage points to current_stage + 1. If current_stage is at end of horizon, next stage directy points to
    // current stage i.e. current_stage = 2 -> next_stage = 3, current_stage = 29 -> next_stage = 29, assuming
    // horizon of 30
    int next_stage = current_stage + (current_stage != solver_->getHorizonLength() - 1);
    // Update tracking point based on predicted distance on track
    mpc_solvers::pacejka_tracking_solvers::trajectory_track_point track_point;
    auto current_state = Eigen::Vector2d(state.pos_x, state.pos_y);
    auto next_track_point = trajectory_->getClosestTrackPoint(current_state);
    track_point.x = next_track_point(0);
    track_point.y = next_track_point(1);


    // Update reference visualization
    last_solution.reference_[current_stage] = Eigen::Vector2d(track_point.x, track_point.y);

    solver_->updateParams(current_stage,
                          model_->getParams(),  // Model Dynamics
                          mpc_costs,            // Costs
                          track_point           // Tracking point
    );

    // Use previous solution as initial guess this time
    solver_->setStateInitialGuess(current_stage, &last_solution.states_[next_stage * solver_->getStateDimension()]);
    solver_->setInputInitialGuess(current_stage, &last_solution.inputs_[next_stage * solver_->getInputDimension()]);
  }
  solver_->solve(&last_solution.states_[0], &last_solution.inputs_[0]);

  last_input_.torque = last_solution.states_[1 * solver_->getStateDimension() + pacejka_tracking_vars::TORQUE];
  last_input_.steer = last_solution.states_[1 * solver_->getStateDimension() + pacejka_tracking_vars::STEER];

  return last_input_;
}

/**
 * @brief Returns the config
 *
 * @return mpc_config&
 */
tracking_mpc_pacejka_config& PacejkaTrackingMpcController::getConfig()
{
  return config_;
}

void PacejkaTrackingMpcController::setConfig(tracking_mpc_pacejka_config config)
{
  config_ = config;
}
};  // namespace crs_controls
