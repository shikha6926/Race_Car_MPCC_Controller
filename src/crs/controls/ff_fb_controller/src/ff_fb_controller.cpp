#include <algorithm>
#include <numeric>
#include <commons/static_track_trajectory.h>

#include "ff_fb_controller/ff_fb_controller.h"

#include "pacejka_model/pacejka_continuous.h"

namespace crs_controls
{
FfFbController::FfFbController(FfFbConfig config, std::shared_ptr<crs_models::pacejka_model::pacejka_params> model,
                               std::shared_ptr<StaticTrackTrajectory> track)
  : ModelBasedController(model, std::static_pointer_cast<Trajectory>(track))
{
  config_.use_filter = false;
  setConfig(config);
  prev_err_angle_ = 0;
  prev_pos_err_ = 0;
  prev_track_angle_ = 0;
  prev_yaw_ = 0;
};

std::shared_ptr<StaticTrackTrajectory> FfFbController::getStaticTrack()
{
  return std::static_pointer_cast<StaticTrackTrajectory>(trajectory_);
}

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

void FfFbController::setConfig(FfFbConfig config)
{
  // Check if filter values have changed
  bool filterChanged = config.use_filter != config_.use_filter;
  filterChanged = filterChanged || config.a_filter != config_.a_filter;
  filterChanged = filterChanged || config.b_filter != config_.b_filter;
  config_ = config;

  if (filterChanged)
  {
    // Recreate filter
    u_steer_filter_ = Filter(config_.b_filter, config_.a_filter);
  }
};

crs_models::pacejka_model::pacejka_car_input FfFbController::getControlInput(
    crs_models::pacejka_model::pacejka_car_state state_input, double timestamp /* Timestamp ignored */)
{
  // absolute speed
  double v = std::sqrt(state_input.vel_x * state_input.vel_x + state_input.vel_y * state_input.vel_y);

  // Speed in world frame
  double vx_w = state_input.vel_x * std::cos(state_input.yaw) - state_input.vel_y * std::sin(state_input.vel_y);
  double vy_w = state_input.vel_x * std::sin(state_input.yaw) + state_input.vel_y * std::cos(state_input.vel_y);

  // find closest point on track to car state
  double lag_compensation_x = state_input.pos_x + vx_w * config_.lag_compensation_time;
  double lag_compensation_y = state_input.pos_y + vy_w * config_.lag_compensation_time;

  // Yaw of the car
  double yaw = wrapToPi(state_input.yaw);
  // Calculate error terms
  auto track_error = getStaticTrack()->getTrackError(Eigen::Vector2d(lag_compensation_x, lag_compensation_y));
  int track_idx = track_error.index;
  int track_side = track_error.side;
  double lat_err = -track_side * track_error.lateral_error;

  // Calculate the reference angle, also update track angles if a full lap occurred
  double track_angle = wrapToPi(getStaticTrack()->getTrackAngle(track_idx));
  if ((track_angle - prev_track_angle_) > M_PI)  // Detect lap change
  {                                              // NOLINT
    getStaticTrack()->increaseTangentAngle();
    track_angle = wrapToPi(getStaticTrack()->getTrackAngle(track_idx));
  }
  else if ((track_angle - prev_track_angle_) < M_PI)  // Detect lap change /
  {                                                   // NOLINT
    getStaticTrack()->decreaseTangentAngle();
    track_angle = wrapToPi(getStaticTrack()->getTrackAngle(track_idx));
  }

  prev_track_angle_ = track_angle;
  prev_yaw_ = yaw;

  double veh_slip_ss = 0;
  double err_angle = yaw - track_angle;
  double err_look_ahead = lat_err + config_.look_ahead_dist * (std::sin(err_angle) + veh_slip_ss);
  double curv = getStaticTrack()->getCurvature(track_idx);
  double d_err_angle = state_input.yaw_rate + v * curv * std::cos(err_angle);

  prev_err_angle_ = err_angle;
  prev_pos_err_ = err_look_ahead;

  // Feedback term
  double u_fb = -config_.Kp * err_look_ahead - config_.Kd * d_err_angle;

  // FEEDFORWARD TERM:
  // dynamic error cancellation:
  double veh_slip = std::atan2(vy_w, vx_w) - yaw;
  veh_slip = 0;
  double side_slip_front_ff = 0;
  double side_slip_rear_ff = veh_slip - model_->lr * state_input.yaw_rate / v;
  if (std::isnan(side_slip_rear_ff))
    side_slip_rear_ff = 0;

  // OTHER FEEDFORWARD:
  double m = model_->m;
  double g = crs_models::pacejka_model::ContinuousPacejkaModel::GRAVITY;
  double C_f = model_->Cf;
  double C_r = model_->Cr;

  double W_f = model_->lr / (model_->lf + model_->lr) * m * g;
  double W_r = model_->lf / (model_->lf + model_->lr) * m * g;
  double K_ug = W_f / C_f - W_r / C_r;
  double u_ff = (model_->lf + model_->lr + K_ug * v * v / g) * curv;

  double u_steer = u_ff + u_fb;

  // FEEDFORWARD APPROXIMATE LONGITUDINAL
  double mean_curv = getStaticTrack()->getMeanCurvatureAlongPath(track_idx, config_.mean_curv_dist);
  double u_torque_ff = config_.target_velocity - config_.K_torque_curv * mean_curv * v * v;

  crs_models::pacejka_model::pacejka_car_input input;

  // Assign values
  // Note internally, we are using normalized torques for the kinmatic model.
  // With this we convert the normalized torque into the real torque command which is required to track a given velocity
  input.torque = (u_torque_ff - config_.b_torque) / config_.a_torque;
  // Saturate torque
  input.torque = std::max(0.0, std::min(1.0, input.torque));

  // Filter steer input
  input.steer = config_.use_filter ? u_steer_filter_.process(u_steer) : u_steer;

  // Saturate steer
  input.steer = std::max(-config_.steer_limit, std::min(config_.steer_limit, input.steer));

  return input;
}

}  // namespace crs_controls
