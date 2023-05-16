#include <ros_estimators/car_estimator/car_estimator.h>

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_discrete.h>
#include <ros_crs_utils/parameter_io.h>
#include <ros_crs_utils/state_message_conversion.h>

namespace ros_estimators
{
template <>
void RosCarEstimator<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input,
                     empty_model>::publishState()
{
  if (has_valid_input_ && model_based_estimator && (base_estimator->getLastValidTs() > 0))
  {
    // Predict up until publishing time if estimator supports it.
    model_based_estimator->controlInputCallback(model_based_estimator->getLastInput(), ros::Time::now().toSec());
  }

  auto msg =
      message_conversion::convertStateToRosMsg<crs_msgs::car_state_cart, crs_models::pacejka_model::pacejka_car_state,
                                               crs_models::pacejka_model::pacejka_car_input>(
          base_estimator->getStateEstimate(), {}  // Pacejka does not need input for message conversion
      );

  if (base_estimator->getLastValidTs() > 0)
    msg.header.stamp = ros::Time(base_estimator->getLastValidTs());
  state_estimate_pub_.publish(msg);
};
}  // namespace ros_estimators