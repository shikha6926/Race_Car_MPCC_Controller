#ifndef ESTIMATORS_MODEL_BASED_ESTIMATOR_H
#define ESTIMATORS_MODEL_BASED_ESTIMATOR_H

#include <estimators/base_estimator.h>

namespace crs_estimators
{
template <typename StateType, typename InputType>
class ModelBasedEstimator : public BaseEstimator<StateType>
{
public:
  /**
   * @brief Construct a new Base Estimator object
   *
   */
  ModelBasedEstimator(){};

  /**
   * @brief Function that gets called when a input was applied
   */
  virtual void controlInputCallback(const InputType input, const double timestamp) = 0;

  /**
   * @brief Returns last valid input that was applied. Can be random if no input has been applied yet.
  */
  virtual InputType getLastInput() const = 0;

};
}  // namespace crs_estimators
#endif /* ESTIMATORS_MODEL_BASED_ESTIMATOR_H */
