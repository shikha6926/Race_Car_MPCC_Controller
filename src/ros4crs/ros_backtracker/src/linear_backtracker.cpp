#include "ros_backtracker/linear_backtracker.h"

namespace ros_backtracker
{

LinearBackTracker::LinearBackTracker(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
  nh_private.getParam("publish_freq", publish_freq_);
  nh_private.getParam("backtrack_distance", backtrack_distance_);
  nh_private.getParam("input_torque", input_torque_);
  nh_private.getParam("max_backtrack_time", max_backtrack_time_);

  input_pub_ = nh_private.advertise<crs_msgs::car_input>("control_input", 1);
  collision_sub_ = nh_private.subscribe("collision", 1, &LinearBackTracker::errorCallback, this);
  state_sub_ = nh_private.subscribe("state", 1, &LinearBackTracker::stateCallback, this);
}

void LinearBackTracker::errorCallback(std_msgs::Bool::ConstPtr error)
{
  /* Crash detected */
  if (error->data)
  {
    error_state = ErrorState::DETECTED;
  }
  /* No crash anymore. If we are in resigned state, we can recover now */
  else if (error_state = ErrorState::RESIGNED)
  {
    error_state = ErrorState::NONE;
  }
}

void LinearBackTracker::stateCallback(crs_msgs::car_state_cart::ConstPtr measured_state)
{
  if (error_state == ErrorState::NONE)
    return;

  // First state message in error state. Save this as reference point
  if (error_state == ErrorState::DETECTED)
  {
    ROS_INFO("Detected collision. Starting Backtracker now!");
    reference_point_ = *measured_state;
    error_state = ErrorState::CONFIRMED;
  }
  else if (error_state == ErrorState::CONFIRMED)
  {
    // Check if we moved enough to leave error state
    if (std::sqrt(std::pow(measured_state->x - reference_point_.x, 2) +
                  std::pow(measured_state->y - reference_point_.y, 2)) >= backtrack_distance_)
    {
      error_state = ErrorState::NONE;  // Error was resolved
    }
    else
    {
      if (reference_point_.header.stamp.toSec() - measured_state->header.stamp.toSec() > max_backtrack_time_)
      {
        ROS_WARN("Backtracker was not able to recover car. Stopping!");
        error_state = ErrorState::RESIGNED;
      }

      crs_msgs::car_input input;
      input.torque = -input_torque_;
      input.steer = 0;
      input.header.stamp = ros::Time::now();
      input_pub_.publish(input);
    }
  }
};
}  // namespace ros_backtracker