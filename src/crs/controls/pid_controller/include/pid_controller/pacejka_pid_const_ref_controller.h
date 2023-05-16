#ifndef PACEJKA_PID_CONST_REF_CONTROLLER
#define PACEJKA_PID_CONST_REF_CONTROLLER

#include "pid_config.h"
#include <commons/filter.h>
#include <commons/trajectory.h>
#include <controls/base_controller.h>
#include <memory>
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>

namespace crs_controls
{
class PacejkaConstRefPIDController
  : public BaseController<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input>
{
private:
  // PID Config
  pid_const_ref_config config_;

  // Errors, used propotional, integral and derivative gain position
  double pos_err_;
  double integral_err_;
  double prev_pos_err_;

  // Errors, used propotional, integral and derivative gain angle
  double angle_err_;
  double integral_angle_err_;
  double prev_angle_err_;

  // Filter used to filter the steer input
  Filter u_steer_filter_;

public:
  /**
   * @brief Constructor, creates a PID controller based on the PID config and a track description
   *
   * @param config PID config
   * @param track The track manager containing all information about the track
   */
  PacejkaConstRefPIDController(pid_const_ref_config config, std::shared_ptr<Trajectory> reference_trajectory);

  /**
   * @brief Executes PID controller, returning a new input to apply
   *
   * @param state current measured state of the system
   * @param timestamp current timestamp, will be ignored from by this pid controller
   * @return crs_models::pacejka_model::pacejka_car_input
   */
  crs_models::pacejka_model::pacejka_car_input
  getControlInput(crs_models::pacejka_model::pacejka_car_state state,
                  double timestamp = 0 /* timestamp will be ignored */) override;

  /**
   * @brief Allows e.g.to update PID gains during runtime, or change target velocity.*
   *
   */
  void setConfig(pid_const_ref_config config);

  /**
   * @brief Returns the pid config
   *
   * @return pid_config&
   */
  pid_const_ref_config& getConfig()
  {
    return config_;
  }

  /**
   * @brief Returns wether the controller is initializing
   *
   * @return true if controller is  initializing
   * @return false if controller is not initializing
   */
  const bool isInitializing() override
  {
    return false;  // PID does not need initializing
  }

  template <typename T> int sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }
};

}  // namespace crs_controls

#endif  // PID_CONTROLLER_PACEJKA_PID_CONTROLLER_H
