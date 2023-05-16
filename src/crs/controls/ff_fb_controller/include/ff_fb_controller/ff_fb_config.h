#ifndef FF_FB_CONTROLLER_FF_FB_CONFIG_H
#define FF_FB_CONTROLLER_FF_FB_CONFIG_H

#include <vector>

namespace crs_controls
{
struct FfFbConfig
{
  // Filtering
  /**
   * @brief Flag to set if filter should be used
   *
   */
  bool use_filter;
  /**
   * @brief Numerator of the discrete filter for the steer angle
   *
   */
  std::vector<double> b_filter;
  /**
   * @brief  Denumerator of the discrete filter for the steer angle
   *
   */
  std::vector<double> a_filter;

  /**
   * @brief Target track velocity
   *
   */
  double target_velocity;
  /**
   * @brief value to comenpensate for lag. Uses a state "lag_compensation_time" in the future instead of the received
   * state
   *
   */
  double lag_compensation_time;

  // torque mapping
  /**
   * @brief Torque mapping to convert crs input to kinematic model torque input
   *
   */
  double a_torque;
  /**
   * @brief Torque mapping to convert crs input to kinematic model torque input
   *
   */
  double b_torque;

  // PID Parameters
  /**
   * @brief Derivative Gain
   *
   */
  double Kd;
  /**
   * @brief Proportional Gain
   *
   */
  double Kp;
  /**
   * @brief Integrator Gain
   *
   */
  double Ki;

  // Limits
  /**
   * @brief The max steer input that can be applied
   *
   */
  double steer_limit;

  /**
   * @brief torque_curv gain
   *
   */
  double K_torque_curv;

  /**
   *  @brief mean distance (track points) to look ahead to calculate track curvature
   *
   */
  int mean_curv_dist;

  /**
   * @brief look ahead distance for reference track angle in feed forward angle
   *
   */
  double look_ahead_dist;
};
}  // namespace crs_controls

#endif  // FF_FB_CONTROLLER_FF_FB_CONFIG_H
