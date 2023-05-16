#include "lowpass_estimator/pacejka_lowpass_estimator.h"

namespace crs_estimators
{
namespace lowpass_estimator
{

/**
 * @brief Measurement update step of the lowpass filter
 *
 * @param data
 */
void PacejkaLowpassEstimator::measurementCallback(const crs_sensor_models::measurement data)
{
  if (data.sensor_key != "vicon")
  {
    std::cout << "[WARN] PacejkaLowpassEstimator can only run with vicon measurements. Got: " << data.sensor_key
              << std::endl;
    return;
  }

  // not initialized
  if (last_valid_ts_ == -1)
  {
    last_valid_ts_ = data.timestamp;

    last_state_world_frame.pos_x = data.measurement_data(0);
    last_state_world_frame.pos_y = data.measurement_data(1);
    last_state_world_frame.yaw = data.measurement_data(2);

    state_filt_world_frame_.pos_x = data.measurement_data(0);
    state_filt_world_frame_.pos_y = data.measurement_data(1);
    state_filt_world_frame_.yaw = data.measurement_data(2);

    state_est_filt_.pos_x = data.measurement_data(0);
    state_est_filt_.pos_y = data.measurement_data(1);
    state_est_filt_.yaw = data.measurement_data(2);
    return;
  }

  double dt = data.timestamp - last_valid_ts_;
  last_valid_ts_ = data.timestamp;

  double measured_x = data.measurement_data(0);
  double measured_y = data.measurement_data(1);
  double measured_yaw = data.measurement_data(2);

  // Update last state
  last_state_world_frame.vel_x = (measured_x - last_state_world_frame.pos_x) / dt;
  last_state_world_frame.vel_y = (measured_y - last_state_world_frame.pos_y) / dt;
  last_state_world_frame.yaw_rate = (measured_yaw - last_state_world_frame.yaw) / dt;
  last_state_world_frame.pos_x = measured_x;
  last_state_world_frame.pos_y = measured_y;
  last_state_world_frame.yaw = measured_yaw;

  // Update filtered state (world frame)
  state_filt_world_frame_.pos_x = last_state_world_frame.pos_x;
  state_filt_world_frame_.pos_y = last_state_world_frame.pos_y;
  state_filt_world_frame_.yaw = yaw_filter.process(last_state_world_frame.yaw);
  state_filt_world_frame_.vel_x = dx_filter.process(last_state_world_frame.vel_x);
  state_filt_world_frame_.vel_y = dy_filter.process(last_state_world_frame.vel_y);
  state_filt_world_frame_.yaw_rate = dyaw_filter.process(last_state_world_frame.yaw_rate);
  // Assign best state. (filtered state in body frame)
  state_est_filt_ = state_filt_world_frame_;
  state_est_filt_.vel_x = state_filt_world_frame_.vel_x * std::cos(state_filt_world_frame_.yaw) +
                          state_filt_world_frame_.vel_y * std::sin(state_filt_world_frame_.yaw);
  state_est_filt_.vel_y = -state_filt_world_frame_.vel_x * std::sin(state_filt_world_frame_.yaw) +
                          state_filt_world_frame_.vel_y * std::cos(state_filt_world_frame_.yaw);
};

}  // namespace lowpass_estimator
}  // namespace crs_estimators
