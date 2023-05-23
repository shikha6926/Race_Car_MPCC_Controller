#include <mpc_controller/pacejka_controller/mpcc_pacejka_controller.h>
// #include <mpc_controller/pacejka_controller/mpcc_pacejka_coef_copy.h>
#include <cmath>
#include <string>
#include <yaml-cpp/yaml.h>
// #include "spline_yaml/fulltrack_spline.yaml"
#ifdef acados_pacejka_mpcc_solver_FOUND
#include "acados_pacejka_mpcc_solver/acados_pacejka_mpcc_solver.h"
#endif


// YAML::Node file_content = YAML::LoadFile("/code/src/crs/controls/mpc_controller/src/pacejka_controller/spline_yaml/fulltrack_spline.yaml");

// std::vector<double> X_coef_0 = file_content["X_d"].as<std::vector<double>>();

// std::vector<double> X_coef_1 = file_content["X_c"].as<std::vector<double>>();
// std::vector<double> X_coef_2 = file_content["X_b"].as<std::vector<double>>();
// std::vector<double> X_coef_3 = file_content["X_a"].as<std::vector<double>>();
// std::vector<double> Y_coef_0 = file_content["Y_d"].as<std::vector<double>>();
// std::vector<double> Y_coef_1 = file_content["Y_c"].as<std::vector<double>>();
// std::vector<double> Y_coef_2 = file_content["Y_b"].as<std::vector<double>>();
// std::vector<double> Y_coef_3 = file_content["Y_a"].as<std::vector<double>>();
// std::vector<double> X_coords = file_content["X_coords"].as<std::vector<double>>();
// std::vector<double> Y_coords = file_content["Y_coords"].as<std::vector<double>>();



namespace crs_controls
{
typedef struct{
  std::vector<double> X_coef_0;
  std::vector<double> X_coef_1;
  std::vector<double> X_coef_2;
  std::vector<double> X_coef_3;
  std::vector<double> Y_coef_0;
  std::vector<double> Y_coef_1;
  std::vector<double> Y_coef_2;
  std::vector<double> Y_coef_3;
  std::vector<double> X_coords;
  std::vector<double> Y_coords;
}SPLINE_PARAMS;

inline bool loadSplineParamsFromFile(std::string yamlFilePath, SPLINE_PARAMS& params)
{
  try
  {
    YAML::Node file_content = YAML::LoadFile(yamlFilePath);
    params.X_coef_0 = file_content["X_d"].as<std::vector<double>>();
    params.X_coef_1 = file_content["X_c"].as<std::vector<double>>();
    params.X_coef_2 = file_content["X_b"].as<std::vector<double>>();
    params.X_coef_3 = file_content["X_a"].as<std::vector<double>>();
    params.Y_coef_0 = file_content["Y_d"].as<std::vector<double>>();
    params.Y_coef_1 = file_content["Y_c"].as<std::vector<double>>();
    params.Y_coef_2 = file_content["Y_b"].as<std::vector<double>>();
    params.Y_coef_3 = file_content["Y_a"].as<std::vector<double>>();
    params.X_coords = file_content["X_coords"].as<std::vector<double>>();
    params.Y_coords = file_content["Y_coords"].as<std::vector<double>>();
  }
  catch (YAML::BadFile& e)
  {
    std::cout << "Error reading file at " << yamlFilePath << e.msg << std::endl;
    return false;
  }
  return true;
}

SPLINE_PARAMS spline_params_obj;
bool read_params_yaml = loadSplineParamsFromFile("/code/src/crs/controls/mpc_controller/src/pacejka_controller/spline_yaml/fulltrack_spline.yaml",
                                                 spline_params_obj);
SplineTrajectory Spline_ref(spline_params_obj.X_coords, spline_params_obj.Y_coords, 
                            spline_params_obj.X_coef_0, spline_params_obj.X_coef_1,
                            spline_params_obj.X_coef_2, spline_params_obj.X_coef_3,
                            spline_params_obj.Y_coef_0, spline_params_obj.Y_coef_1, 
                            spline_params_obj.Y_coef_2, spline_params_obj.Y_coef_3);

void PacejkaMpccController::loadMpcSolver()
{
  if (config_.solver_type == "ACADOS")
  {
#ifdef acados_pacejka_mpcc_solver_FOUND
    solver_ = std::make_shared<mpc_solvers::pacejka_solvers::AcadosPacejkaMpccSolver>();
#else
    assert(true && "Requested ACADOS solver for pacejka MPC. Solver not found!");
#endif
  }
}

std::shared_ptr<StaticTrackTrajectory> PacejkaMpccController::getStaticTrack()
{
  return std::static_pointer_cast<StaticTrackTrajectory>(trajectory_);
}

// Constructor, Creates a MPC controller based on the config and a track description
PacejkaMpccController::PacejkaMpccController(mpcc_pacejka_config config,
                                             std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
                                             std::shared_ptr<StaticTrackTrajectory> track)
  : MpcController(model, std::static_pointer_cast<Trajectory>(track))
{
  setConfig(config);
  loadMpcSolver();

  // Initialize last solutions with zeros
  last_solution.states_ = std::vector<double>(solver_->getStateDimension() * solver_->getHorizonLength(), 0.0);
  last_solution.inputs_ = std::vector<double>(solver_->getInputDimension() * solver_->getHorizonLength(), 0.0);

  // Initialize last references with zeros (only for visualization)
  for (int i = 0; i < solver_->getHorizonLength(); i++)
    last_solution.reference_on_track_.push_back(Eigen::Vector3d(0, 0, 0));
}

std::vector<std::vector<double>> PacejkaMpccController::getPlannedTrajectory()
{
  std::vector<std::vector<double>> traj;
  for (int i = 0; i < solver_->getHorizonLength(); i++)
  {
    double x_pos = last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::X];
    double y_pos = last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::Y];
    double vel = std::sqrt(std::pow(last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::VX], 2) +
                           std::pow(last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::VY], 2));
    // std::cout<< "planned x:"<<x_pos <<"\n";
    // std::cout<< "planned y:"<<y_pos <<"\n";
    // Append values to trajectory for visualization
    traj.push_back({
        x_pos,                                                                        // Planned x
        y_pos,                                                                        // Planned y
        vel,                                                                          // Planned velocity
        last_solution.states_[i * solver_->getStateDimension() + pacejka_vars::YAW],  // Planned Yaw
        last_solution.reference_on_track_[i].x(),                                     // Reference x
        last_solution.reference_on_track_[i].y(),                                     // Reference y
        last_solution.reference_on_track_[i](2)                                       // Reference yaw
    });
  }

  return traj;
}

void PacejkaMpccController::initialize(crs_models::pacejka_model::pacejka_car_state state)
{
  int N_ = solver_->getHorizonLength();
  double max_arc_length = Spline_ref.getMaxArcLength();
  auto model_dynamics = model_->getParams();
  int N_WARM_START = 1000;

  mpc_solvers::pacejka_solvers::tracking_costs mpc_costs = { config_.Q1, config_.Q2, config_.R1,
                                                             config_.R2, config_.R3, config_.q };
  // Initial inputs.
  last_input_.steer = 0;
  // Initial torque input. Lets use 50% throttle
  last_input_.torque = 0.5;
  // Set initial arc length
  theta_ = getStaticTrack()->getArcLength(
      getStaticTrack()->getClosestTrackPointIdx(Eigen::Vector2d(state.pos_x, state.pos_y)));

  // int Spline_index = Spline_ref.getclosestsplineindex(Eigen::Vector2d(state.pos_x, state.pos_y), Spline_ref);
  // theta_ = Spline_ref.gettheta(Spline_index, Eigen::Vector2d(state.pos_x, state.pos_y), Spline_ref);

  double x_init[9];
  x_init[0] = state.pos_x;
  x_init[1] = state.pos_y;
  x_init[2] = state.yaw;
  x_init[3] = std::max(0.01, state.vel_x);  // Should not be zero as otherwise the model gets NaN issues
  x_init[4] = state.vel_y;
  x_init[5] = state.yaw_rate;
  x_init[6] = last_input_.torque;
  x_init[7] = last_input_.steer;
  x_init[8] = theta_;

  solver_->setInitialState(x_init);

  double u0[solver_->getStateDimension()] = { 0.0, last_input_.torque };

  // Setup parameter for initial solve
  for (int stage = 0; stage < solver_->getHorizonLength(); stage++)
  {
    // Set initial state
    for (int x_idx = 0; x_idx < solver_->getStateDimension(); x_idx++)
      last_solution.states_[stage * solver_->getStateDimension() + x_idx] = x_init[x_idx];

    // Set initial input (nothing to do, its zero)
  }

  // Run solver
  for (int run = 0; run < N_WARM_START; run++)
  {
    for (int stage = 0; stage < solver_->getHorizonLength(); stage++)
    {
      
      // Ugly indexing to get distance on track of predicted solution
      double distance_on_track = last_solution.states_[stage * solver_->getStateDimension() + pacejka_vars::THETA];
      // Convert it to track indice using density of track points which are regularly sampled
    
      // Update tracking point based on predicted distance on track
      mpc_solvers::pacejka_solvers::trajectory_track_point track_point;
     
      track_point.x = Spline_ref.getRefCoords(distance_on_track).x();
      track_point.y = Spline_ref.getRefCoords(distance_on_track).y();
      track_point.grad_x = Spline_ref.getGradient(distance_on_track).x();
      track_point.grad_y = Spline_ref.getGradient(distance_on_track).y();
      track_point.theta = distance_on_track;
      track_point.phi = Spline_ref.getTangentAngle(track_point.grad_y, track_point.grad_x);

      // Update reference visualization
      last_solution.reference_on_track_[stage] = Eigen::Vector3d(track_point.x, track_point.y, track_point.phi);

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
crs_models::pacejka_model::pacejka_car_input PacejkaMpccController::getControlInput(
    crs_models::pacejka_model::pacejka_car_state state, double timestamp /* timestamp will be ignored */)
{
  int N_ = solver_->getHorizonLength();
  // max_arc_length is needed to detect when a lap is done. The full track has
  // been extended by a full lap, such that the horizon can predict past the
  // start
  double max_arc_length = Spline_ref.getMaxArcLength();
  mpc_solvers::pacejka_solvers::tracking_costs mpc_costs = { config_.Q1, config_.Q2, config_.R1,
                                                             config_.R2, config_.R3, config_.q };

  if (!is_initialized)
  {
    initializing = true;
    initialize(state);
    initializing = false;
    is_initialized = true;
  }

  // initialize state to virtually advanced vehicle states and inputs
  auto virtual_state = model_->applyModel(state, last_input_, config_.lag_compensation_time);
  
  // theta_ = getStaticTrack()->getArcLength(
  //     getStaticTrack()->getClosestTrackPointIdx(Eigen::Vector2d(virtual_state.pos_x, virtual_state.pos_y)));

  double x0[] = {
    virtual_state.pos_x, virtual_state.pos_y, virtual_state.yaw,
    virtual_state.vel_x, virtual_state.vel_y, virtual_state.yaw_rate,
    last_input_.torque,  last_input_.steer,   theta_,
  };

  // if lap is done, increase lap counter
  if (theta_ > (laps_ + 1) * max_arc_length)
  {
    laps_++;
  }

  solver_->setInitialState(x0);
  // Run solver
  for (int current_stage = 0; current_stage < solver_->getHorizonLength(); current_stage++)
  {
    // next stage points to current_stage + 1. If current_stage is at end of horizon, next stage directy points to
    // current stage i.e. current_stage = 2 -> next_stage = 3, current_stage = 29 -> next_stage = 29, assuming
    // horizon of 30
    int next_stage = current_stage + (current_stage != solver_->getHorizonLength() - 1);
    // Ugly indexing to get distance on track of predicted solution
    double distance_on_track = last_solution.states_[next_stage * solver_->getStateDimension() + pacejka_vars::THETA] -
                               laps_ * Spline_ref.getMaxArcLength();
    std::cout << "Distance on track:" << distance_on_track << "\n";
    // Convert it to track indice using density of track points which are regularly sampled
    //int reference_track_index = distance_on_track * getStaticTrack()->getDensity();
    // Update tracking point based on predicted distance on track
    mpc_solvers::pacejka_solvers::trajectory_track_point track_point;
    //track_point.x = getStaticTrack()->operator[](reference_track_index).x();
    //track_point.y = getStaticTrack()->operator[](reference_track_index).y();
    //track_point.grad_x = getStaticTrack()->getRate(reference_track_index).x();
    //track_point.grad_y = getStaticTrack()->getRate(reference_track_index).y();
    //track_point.theta = distance_on_track + laps_ * getStaticTrack()->getMaxArcLength();
    //track_point.phi = getStaticTrack()->getTrackAngle(reference_track_index) + laps_ * 2 * M_PI;
  
    track_point.x = Spline_ref.getRefCoords(distance_on_track).x();
    track_point.y = Spline_ref.getRefCoords(distance_on_track).y();
    track_point.grad_x = Spline_ref.getGradient(distance_on_track).x();
    track_point.grad_y = Spline_ref.getGradient(distance_on_track).y();
    track_point.theta = distance_on_track + laps_ * Spline_ref.getMaxArcLength();
    track_point.phi = Spline_ref.getTangentAngle(track_point.grad_y, track_point.grad_x); + laps_ * 2 * M_PI;
    std::cout<< "Track point x: "<< track_point.x << "\n";
    std::cout<< "Track point y: "<< track_point.y << "\n";
    // Update reference visualization
    last_solution.reference_on_track_[current_stage] = Eigen::Vector3d(track_point.x, track_point.y, track_point.phi);

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

  last_input_.torque = last_solution.states_[1 * solver_->getStateDimension() + pacejka_vars::TORQUE];
  last_input_.steer = last_solution.states_[1 * solver_->getStateDimension() + pacejka_vars::STEER];
  theta_ = last_solution.states_[1 * solver_->getStateDimension() + pacejka_vars::THETA];

  return last_input_;
}

/**
 * @brief Returns the config
 *
 * @return mpc_config&
 */
mpcc_pacejka_config& PacejkaMpccController::getConfig()
{
  return config_;
}

void PacejkaMpccController::setConfig(mpcc_pacejka_config config)
{
  config_ = config;
}
};  // namespace crs_controls
