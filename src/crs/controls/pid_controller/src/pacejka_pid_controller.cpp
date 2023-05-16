#include "pid_controller/pacejka_pid_controller.h"
#include <algorithm>
#include <numeric>

namespace crs_controls
{

PacejkaPIDController::PacejkaPIDController(pid_config config, std::shared_ptr<StaticTrackTrajectory> track)
  : BaseController(std::static_pointer_cast<Trajectory>(track))
{
  setConfig(config);
};

void PacejkaPIDController::setConfig(pid_config config)
{
  // Check if filter values have changed. If yes, create new filter
  bool filterChanged = config.use_filter != config_.use_filter;
  filterChanged = filterChanged || config.a_filter != config_.a_filter;
  filterChanged = filterChanged || config.b_filter != config_.b_filter;

  if (filterChanged)
  {
    u_steer_filter_ = Filter(config.b_filter, config.a_filter);
  }
  // Update config
  config_ = config;
};

crs_models::pacejka_model::pacejka_car_input PacejkaPIDController::getControlInput(
    crs_models::pacejka_model::pacejka_car_state state_input, double timestamp /* ignored */)
{
  double vx_w = state_input.vel_x * std::cos(state_input.yaw) - state_input.vel_y * std::sin(state_input.vel_y);
  double vy_w = state_input.vel_x * std::sin(state_input.yaw) + state_input.vel_y * std::cos(state_input.vel_y);
  // look ahead - to compensate for lag in communication
  double look_ahead_x = state_input.pos_x + vx_w * config_.lag_compensation_time;
  double look_ahead_y = state_input.pos_y + vy_w * config_.lag_compensation_time;

  auto track_error = getTrajectory<StaticTrackTrajectory>()->getTrackError(Eigen::Vector2d(look_ahead_x, look_ahead_y));
  int track_idx = track_error.index;
  int track_side = track_error.side;

  // Basic PID control law
  pos_err_ = track_side * track_error.lateral_error;
  integral_err_ += pos_err_;

  double u_steer = config_.Kp * pos_err_ + config_.Kd * (pos_err_ - prev_pos_err_) + config_.Ki * integral_err_;

  prev_pos_err_ = pos_err_;  // update delayed error term

  // Apply filtering on input (if enabled) and convert the normalized internal torque to a
  // real torque command in order to track a given velocity.
  u_steer = config_.use_filter ? u_steer_filter_.process(u_steer) : u_steer;
  double u_torque = (config_.target_velocity - config_.b_torque) / config_.a_torque;

  // Assign saturated values to control input
  crs_models::pacejka_model::pacejka_car_input input = {
    .torque = std::clamp(u_torque, 0.0, 1.0),
    .steer = std::clamp(u_steer, -config_.steer_limit, +config_.steer_limit)
  };

  return input;
}

}  // namespace crs_controls
