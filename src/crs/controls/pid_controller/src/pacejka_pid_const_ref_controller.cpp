#include "pid_controller/pacejka_pid_const_ref_controller.h"
#include <algorithm>
#include <numeric>

namespace crs_controls
{

PacejkaConstRefPIDController::PacejkaConstRefPIDController(pid_const_ref_config config,
                                                           std::shared_ptr<Trajectory> reference_trajectory)
  : BaseController(reference_trajectory)
{
  setConfig(config);
};

void PacejkaConstRefPIDController::setConfig(pid_const_ref_config config)
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
crs_models::pacejka_model::pacejka_car_input PacejkaConstRefPIDController::getControlInput(
    crs_models::pacejka_model::pacejka_car_state state_input, double timestamp /* ignored */)
{
  double vx_w = state_input.vel_x * std::cos(state_input.yaw) - state_input.vel_y * std::sin(state_input.vel_y);
  double vy_w = state_input.vel_x * std::sin(state_input.yaw) + state_input.vel_y * std::cos(state_input.vel_y);
  // look ahead - to compensate for lag in communication
  double look_ahead_x = state_input.pos_x + vx_w * config_.lag_compensation_time;
  double look_ahead_y = state_input.pos_y + vy_w * config_.lag_compensation_time;

  auto look_ahead_state = Eigen::Vector2d(look_ahead_x, look_ahead_y);
  auto track_point = trajectory_->getClosestTrackPoint(look_ahead_state);

  // Calculate Errors in position
  auto error_vec = track_point - look_ahead_state;
  auto error_mag = error_vec.norm();

  pos_err_ = error_mag;
  integral_err_ += pos_err_;

  // Angle errors
  double wraped_yaw = wrapToPi(state_input.yaw);
  int sign_wraped_yaw = sgn(wraped_yaw);
  double angletox_error_vec = std::atan2(error_vec.y(), error_vec.x());

  double error_angle = angletox_error_vec - wraped_yaw;
  double error_angle_2 = sign_wraped_yaw*(2*M_PI) + angletox_error_vec - wraped_yaw;

  if(std::abs(error_angle) < std::abs(error_angle_2))
  {
    angle_err_ = error_angle;
  }
  else
  {
    angle_err_ = error_angle_2;
  }
  integral_angle_err_ += angle_err_;


  double u_torque = config_.Kp * pos_err_ + config_.Kd * (pos_err_ - prev_pos_err_) + config_.Ki * integral_err_;
  double u_steer = config_.Kp_angle * angle_err_ + config_.Kd_angle * (angle_err_ - prev_angle_err_) + config_.Ki_angle * integral_angle_err_;

  // update delayed error terms
  prev_pos_err_ = pos_err_;
  prev_angle_err_ = angle_err_;


  // Apply filtering on input (if enabled) and convert the normalized internal torque to a
  // real torque command in order to track a given velocity.
  u_torque = (u_torque - config_.b_torque) / config_.a_torque;
  u_steer = config_.use_filter ? u_steer_filter_.process(u_steer) : u_steer;

  // Assign saturated values to control input
  crs_models::pacejka_model::pacejka_car_input input = {
    .torque = std::clamp(u_torque, 0.02, 0.3),
    .steer = std::clamp(u_steer, -config_.steer_limit, +config_.steer_limit)
  };

  return input;
}

}  // namespace crs_controls
