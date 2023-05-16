#include <ros_estimators/car_estimator/car_estimator.h>

#include <kinematic_model/kinematic_car_input.h>
#include <kinematic_model/kinematic_car_state.h>
#include <kinematic_model/kinematic_discrete.h>
#include <ros_crs_utils/parameter_io.h>
#include <ros_crs_utils/state_message_conversion.h>
#include <ros_crs_utils/state_message_conversion.h>

namespace ros_estimators
{
template <>
void RosCarEstimator<crs_models::kinematic_model::kinematic_car_state, crs_models::kinematic_model::kinematic_car_input,
                     crs_models::kinematic_model::kinematic_params>::publishState()
{
  auto msg = message_conversion::convertStateToRosMsg<crs_msgs::car_state_cart,
                                                      crs_models::kinematic_model::kinematic_car_state,
                                                      message_conversion::kinematic_information>(
      base_estimator->getStateEstimate(),
      { message_conversion::convertToCrsInput<crs_msgs::car_input, crs_models::kinematic_model::kinematic_car_input>(
            last_input_),
        *model });

  if (base_estimator->getLastValidTs() > 0)
    msg.header.stamp = ros::Time(base_estimator->getLastValidTs());
  state_estimate_pub_.publish(msg);
};

}  // namespace ros_estimators