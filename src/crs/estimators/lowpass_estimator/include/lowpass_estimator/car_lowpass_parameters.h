#ifndef LOWPASS_ESTIMATOR_CAR_LOWPASS_PARAMETERS_H
#define LOWPASS_ESTIMATOR_CAR_LOWPASS_PARAMETERS_H
#include <vector>

namespace crs_estimators
{
namespace lowpass_estimator
{
struct car_lowpass_parameters
{
  /**
   * @brief Filter numerator coefficient for velocity x
   *
   */
  std::vector<double> b_dx;

  /**
   * @brief Filter denumerator coefficient for velocity x
   *
   */
  std::vector<double> a_dx;

  /**
   * @brief Filter numerator coefficient for velocity y
   *
   */
  std::vector<double> b_dy;

  /**
   * @brief Filter denumerator coefficient for velocity y
   *
   */
  std::vector<double> a_dy;

  /**
   * @brief Filter numerator coefficient for yaw x
   *
   */
  std::vector<double> b_yaw;

  /**
   * @brief Filter denumerator coefficient for yaw x
   *
   */
  std::vector<double> a_yaw;

  /**
   * @brief Filter numerator coefficient for yaw rate x
   *
   */
  std::vector<double> b_dyaw;

  /**
   * @brief Filter denumerator coefficient for yaw rate
   *
   */
  std::vector<double> a_dyaw;
};
}  // namespace lowpass_estimator
}  // namespace crs_estimators

#endif  // LOWPASS_ESTIMATOR_CAR_LOWPASS_PARAMETERS_H
