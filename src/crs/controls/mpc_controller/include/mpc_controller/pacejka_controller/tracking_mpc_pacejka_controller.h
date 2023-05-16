#ifndef MPC_CONTROLLER_PACEJKA_CONTROLLER_TRACKING_MPC_PACEJKA_CONTROLLER_H
#define MPC_CONTROLLER_PACEJKA_CONTROLLER_TRACKING_MPC_PACEJKA_CONTROLLER_H

#include <commons/filter.h>
#include <controls/mpc_controller.h>
#include <commons/trajectory.h>
#include <memory>

#include <cmath>
#include <thread>

#include "tracking_mpc_pacejka_config.h"
#include <pacejka_model/pacejka_discrete.h>
#include <mpc_solvers/pacejka_tracking_mpc_solver.h>

typedef mpc_solvers::pacejka_tracking_solvers::vars pacejka_tracking_vars;
typedef mpc_solvers::pacejka_tracking_solvers::inputs pacejka_tracking_inputs;

namespace crs_controls
{
/**
 * @brief The predicted mpc solution
 *
 */
struct tracking_mpc_solution
{
  /**
   * @brief The predicted states over the full horizon.
   * @note Dimension is number_of_states * horizon_length
   *
   */
  std::vector<double> states_;

  /**
   * @brief The predicted input over the full horizon.
   * @note Dimension is number_of_inputs * horizon_length
   *
   * This is not the car input (these are part of the states vector), but rather the rate of change of the car inputs
   * as well as rate of change of the track distance
   */
  std::vector<double> inputs_;
  std::vector<Eigen::Vector2d> reference_;
};

class PacejkaTrackingMpcController
  : public MpcController<crs_models::pacejka_model::DiscretePacejkaModel, crs_models::pacejka_model::pacejka_car_state,
                         crs_models::pacejka_model::pacejka_car_input>
{
private:
  // The MPC config (cost values)
  tracking_mpc_pacejka_config config_;
  // Shared pointer to the solver for the MPC problem (requires changing)
  std::shared_ptr<mpc_solvers::pacejka_tracking_solvers::PacejkaTrackingMpcSolver> solver_;
  // Current distance driven on the track (not used)
  //double theta_ = 0.0;

  bool is_initialized = false;
  bool initializing = false;

  // Reference to last mpc solution
  tracking_mpc_solution last_solution;

  // Pointer to the thread that currently solves the optimization problem
  std::thread* solver_thread_;

  // Last known input that was applied
  crs_models::pacejka_model::pacejka_car_input last_input_;

  // Current lap count (not used)
  //int laps_ = 0;

  //std::shared_ptr<StaticTrackTrajectory> getStaticTrack();
  void loadMpcSolver();

public:
  // Constructor, Creates a MPC controller based on the config and a track description (requires changing of pointer to dynamic track)
  PacejkaTrackingMpcController(tracking_mpc_pacejka_config config,
                        std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
                        std::shared_ptr<Trajectory> reference_trajectory);
  /**
   * @brief Get the Planned Trajectory.
   * Returns a vector with 4 entries for each step of the horizon
   *
   * @return std::vector<Eigen::VectorXd> vector containing
   * [planned_x,
   *  planned_y,
   *  planned_velocity,
   *  planned_yaw,
   * ]
   */
  std::vector<std::vector<double>> getPlannedTrajectory() override;

  /**
   * @brief Initializes the mpc solver
   *
   */
  void initialize(crs_models::pacejka_model::pacejka_car_state state);

  /**
   * @brief Executes the MPC controller, returning a new input to apply
   *
   * @param state current measured state of the system
   * @param timestamp current timestamp, will be ignored
   * @return crs_models::pacejka_model::pacejka_car_input
   */
  crs_models::pacejka_model::pacejka_car_input
  getControlInput(crs_models::pacejka_model::pacejka_car_state state,
                  double timestamp = 0 /* timestamp will be ignored */) override;

  /**
   * @brief Updates the mpc config
   *
   */
  void setConfig(tracking_mpc_pacejka_config config);

  /**
   * @brief Returns the config
   *
   * @return mpc_config&
   */
  tracking_mpc_pacejka_config& getConfig();

  /**
   * @brief Returns wether the controller is initialized
   *
   * @return true if controller is already initialized
   * @return false if controller is not initialized
   */
  const bool isInitializing() override
  {
    return initializing;
  }
};

}  // namespace crs_controls

#endif
