#ifndef SRC_CRS_CONTROLS_PID_CONTROLLER_INCLUDE_PID_CONTROLLER_PID_CONFIG
#define SRC_CRS_CONTROLS_PID_CONTROLLER_INCLUDE_PID_CONTROLLER_PID_CONFIG

#include <vector>
namespace crs_controls
{
struct pid_config
{
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

  // Filtering
  /**
   * @brief Flag to set if filter should be used
   *
   */
  bool use_filter = false;
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
};

struct pid_const_ref_config
{
  /**
   * @brief value to comenpensate for lag. Uses a state "lag_compensation_time" in the future instead of the received
   * state
   *
   */
  double lag_compensation_time;

  // Filtering
  /**
   * @brief Flag to set if filter should be used
   *
   */
  bool use_filter = false;
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
   * @brief Derivative Gain Position
   *
   */
  double Kd;
  /**
   * @brief Proportional Gain Position
   *
   */
  double Kp;
  /**
   * @brief Integrator Gain Positiontion
   *
   */
  double Ki;

  /**
   * @brief Derivative Gain Angle
   *
   */
  double Kd_angle;
  /**
   * @brief Proportional Gain Angle
   *
   */
  double Kp_angle;
  /**
   * @brief Integrator Gain Angle
   *
   */
  double Ki_angle;

  // Limits
  /**
   * @brief The max steer input that can be applied
   *
   */
  double steer_limit;
};
}  // namespace crs_controls

#endif /* SRC_CRS_CONTROLS_PID_CONTROLLER_INCLUDE_PID_CONTROLLER_PID_CONFIG */
