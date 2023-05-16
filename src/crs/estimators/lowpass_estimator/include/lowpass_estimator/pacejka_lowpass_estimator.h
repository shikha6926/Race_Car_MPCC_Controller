#ifndef LOWPASS_ESTIMATOR_PACEJKA_LOWPASS_ESTIMATOR_H
#define LOWPASS_ESTIMATOR_PACEJKA_LOWPASS_ESTIMATOR_H

#include "lowpass_estimator/lowpass_estimator.h"

#include "car_lowpass_parameters.h"
#include <commons/filter.h>
#include <pacejka_model/pacejka_car_state.h>

namespace crs_estimators
{
namespace lowpass_estimator
{
class PacejkaLowpassEstimator : public LowpassEstimator<crs_models::pacejka_model::pacejka_car_state>
{
private:
  Filter dx_filter;
  Filter dy_filter;
  Filter yaw_filter;
  Filter dyaw_filter;

  crs_models::pacejka_model::pacejka_car_state last_state_world_frame;
  crs_models::pacejka_model::pacejka_car_state state_filt_world_frame_;

public: 
  // Constructor
  PacejkaLowpassEstimator(const car_lowpass_parameters& params,
                          const crs_models::pacejka_model::pacejka_car_state initial_state)
    : LowpassEstimator<crs_models::pacejka_model::pacejka_car_state>(initial_state)
    , dx_filter(params.b_dx, params.a_dx)
    , dy_filter(params.b_dy, params.a_dy)
    , yaw_filter(params.b_yaw, params.a_yaw)
    , dyaw_filter(params.b_dyaw, params.a_dyaw)
  {
    last_state_world_frame = initial_state;
    last_state_world_frame.vel_x =
        initial_state.vel_x * std::sin(initial_state.yaw) - initial_state.vel_y * std::cos(initial_state.yaw);
    last_state_world_frame.vel_y =
        initial_state.vel_x * std::cos(initial_state.yaw) + initial_state.vel_y * std::sin(initial_state.yaw);
    state_filt_world_frame_ = last_state_world_frame;
  };
  /**
   * @brief This is the measurement update step of the Estimator.
   *        The state prediction based on the model is updated using measurement inofrmation.
   *
   * @param data measurement information, e.g. type of sensor, measurement data
   */
  void measurementCallback(const crs_sensor_models::measurement measurement) override;
  
};
}  // namespace lowpass_estimator
}  // namespace crs_estimators
#endif
