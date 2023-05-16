#ifndef ROS_ESTIMATORS_STATE_ESTIMATOR_ROS_H
#define ROS_ESTIMATORS_STATE_ESTIMATOR_ROS_H

namespace ros_estimators
{
class RosStateEstimator
{
public:
  virtual void publishState() = 0;
};
}  // namespace ros_estimators
#endif
