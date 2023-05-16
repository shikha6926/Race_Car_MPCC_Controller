#ifndef LOWPASS_ESTIMATOR_LOWPASS_ESTIMATOR_H
#define LOWPASS_ESTIMATOR_LOWPASS_ESTIMATOR_H

#include <estimators/base_estimator.h>
#include <sensor_models/sensor_measurement.h>

namespace crs_estimators
{
namespace lowpass_estimator
{
template <typename StateType>
class LowpassEstimator : public BaseEstimator<StateType>
{ 
public:
  // Constructor
  LowpassEstimator(const StateType initial_state) : state_est_filt_(initial_state){};


  StateType getStateEstimate() const override
  {
    // return the current state estimate
    return state_est_filt_;
  }


  void setState(StateType state)
  {
    state_est_filt_ = state;
  }

  double getLastValidTs() const override {
    return last_valid_ts_;
  };

protected:
  StateType state_est_filt_;
  double last_valid_ts_ = -1;
};
}  // namespace lowpass_estimator
}  // namespace crs_estimators
#endif
