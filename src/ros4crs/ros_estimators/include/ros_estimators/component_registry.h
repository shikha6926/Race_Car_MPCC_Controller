#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY
#include "ros_estimators/state_estimator_ros.h"
#include "ros_estimators/car_estimator/car_estimator.h"
#include <ros/ros.h>

namespace ros_estimators
{

RosStateEstimator* resolveEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& state_type,
                                    const std::string& input_type, const std::string& estimator_type);

template <typename StateType, typename InputType, typename ModelType = ros_estimators::empty_model>
RosCarEstimator<StateType, InputType, ModelType>* resolveCarEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                                      const std::string& estimator_type);
}  // namespace ros_estimators
#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_COMPONENT_REGISTRY */
