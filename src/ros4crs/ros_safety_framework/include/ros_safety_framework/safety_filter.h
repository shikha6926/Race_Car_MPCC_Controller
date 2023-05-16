#ifndef SRC_ROS_ROS_SAFETY_FRAMEWORK_INCLUDE_ROS_SAFETY_FRAMEWORK_SAFETY_FILTER
#define SRC_ROS_ROS_SAFETY_FRAMEWORK_INCLUDE_ROS_SAFETY_FRAMEWORK_SAFETY_FILTER

#include <commons/static_track_trajectory.h>
#include <crs_msgs/car_state_cart.h>
#include <numeric>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <safety_framework/safety_filter.h>
#include <std_srvs/SetBool.h>

#include <ros_crs_utils/state_message_conversion.h>

namespace ros_safety
{
template <typename StateType, typename InputType, typename StateMsg, typename InputMsg>
class SafetyFilter
{
private:
  std::shared_ptr<crs_safety::SafetyFilter<StateType, InputType>> safety_filter_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishes collision messages
  ros::Publisher input_pub_;

  ros::Subscriber state_sub_;
  ros::Subscriber input_sub_;

  ros::ServiceServer running_service_;

  bool running_ = true;

  boost::shared_ptr<StateMsg const> last_state_msg_;

public:
  /**
   * @brief Construct a new Crash Detector object
   *
   * @param nh public nodehandle
   * @param nh_private  private nodehandle
   * @param static_track_trajectory track manager used to find collisions
   */
  SafetyFilter(ros::NodeHandle nh, ros::NodeHandle nh_private,
               std::shared_ptr<crs_safety::SafetyFilter<StateType, InputType>> safety_filter)
    : nh_(nh), nh_private_(nh_private), safety_filter_(safety_filter)
  {
    running_service_ = nh_private_.advertiseService("set_running", &SafetyFilter::setRunningCallback, this);
    state_sub_ = nh_private_.subscribe<const boost::shared_ptr<StateMsg const>&>("state", 10,
                                                                                 &SafetyFilter::stateCallback, this);
    input_sub_ = nh_private_.subscribe<const boost::shared_ptr<InputMsg const>&>("input", 10,
                                                                                 &SafetyFilter::inputCallback, this);
    input_pub_ = nh_private_.advertise<InputMsg>("save_input", 10);
  }

  void stateCallback(const boost::shared_ptr<StateMsg const>& state_msg)
  {
    last_state_msg_ = state_msg;
  }
  void inputCallback(const boost::shared_ptr<InputMsg const>& input_msg)
  {
    if (!running_)
    {
      input_pub_.publish(input_msg);
      return;
    }

    StateType state = message_conversion::convertMsgToState<StateType, StateMsg>(*last_state_msg_);
    InputType input = message_conversion::convertToCrsInput<InputMsg, InputType>(*input_msg);

    InputType safe_input = safety_filter_->getSafeControlInput(state, input);
    input_pub_.publish(message_conversion::convertToRosInput<InputMsg, InputType>(safe_input));
  }

  bool setRunningCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    running_ = req.data;
    res.success = true;
    return true;
  }
};
}  // namespace ros_safety

#endif  // ROS_BACKTRACKER_CRASH_DETECTOR_H
