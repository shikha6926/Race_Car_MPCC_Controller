#ifndef ESTIMATORS_BASE_ESTIMATOR_H
#define ESTIMATORS_BASE_ESTIMATOR_H

#include <sensor_models/sensor_measurement.h>
#include <memory>

namespace crs_estimators
{
template <typename StateType>
class BaseEstimator
{
public:
  /**
   * @brief Construct a new Base Estimator object
   *
   */
  BaseEstimator(){};

  /**
   * Returns the currently best state estimate
   */
  virtual StateType getStateEstimate() const = 0;

  /**
   * @brief Get the last valid timestamp (i.e. timestamp for which the state estimate is valid)
   *
   * @return double
   */
  virtual double getLastValidTs() const = 0;

  /**
   * @brief Function that gets called when new measurements from sensor models arrive
   *
   */
  virtual void measurementCallback(const crs_sensor_models::measurement measurement) = 0;
  
};
}  // namespace crs_estimators
#endif /* ESTIMATORS_BASE_ESTIMATOR_H */
