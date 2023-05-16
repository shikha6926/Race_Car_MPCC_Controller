#include "ros_backtracker/crash_detector.h"

namespace ros_backtracker
{
CrashDetector::CrashDetector(ros::NodeHandle nh, ros::NodeHandle nh_private,
                             std::shared_ptr<crs_controls::StaticTrackTrajectory> static_track_trajectory)
  : static_track_trajectory_(static_track_trajectory), nh_(nh), nh_private_(nh_private)
{
  loadParams();
  collision_pub_ = nh.advertise<std_msgs::Bool>("collision", 1);
  state_sub_ = nh.subscribe("state", 1, &CrashDetector::callback, this);
}

void CrashDetector::loadParams()
{
  nh_private_.getParam("distance_tolerance", distance_tolerance_);
  nh_private_.getParam("time_tolerance", time_tolerance_);
  nh_private_.getParam("min_move_distance", min_move_distance_);
  nh_private_.getParam("check_rate", check_rate_);
  nh_private_.getParam("collision_msg_rate", collision_msg_rate_);
}

void CrashDetector::callback(crs_msgs::car_state_cart::ConstPtr measured_state)
{
  if (measured_state->header.stamp.toSec() - last_message_ts_ < 1 / check_rate_)
    return;
  last_message_ts_ = measured_state->header.stamp.toSec();
  Eigen::Vector2d position = Eigen::Vector2d(measured_state->x, measured_state->y);

  // Check if car is not on track anymore
  auto distance_to_center_line = static_track_trajectory_->getTrackError(position).lateral_error;
  bool track_collision = distance_to_center_line > (static_track_trajectory_->getWidth() / 2 + distance_tolerance_);

  // Check if car is stuck somewhere
  bool valid_state = false;
  crs_msgs::car_state_cart state;
  last_states_.push_back(*measured_state);
  while (last_message_ts_ - last_states_.front().header.stamp.toSec() >= time_tolerance_)
  {
    state = last_states_.front();
    last_states_.pop_front();
    valid_state = true;
  }
  bool stand_still =
      valid_state &&
      (std::sqrt(std::pow(state.x - position.x(), 2) + std::pow(state.y - position.y(), 2)) < min_move_distance_);
  bool collision_detected_now = track_collision || stand_still;

  if (collision_detected_now && (collision_detected_now == collision_detected_))  // collision state did not change
  {
    // check rate to not flood topic with messages
    if (last_message_ts_ - last_collision_msg_ts_ < 1 / collision_msg_rate_)
      return;
  }
  last_collision_msg_ts_ = last_message_ts_;
  collision_detected_ = collision_detected_now;

  std_msgs::Bool msg;
  msg.data = collision_detected_;
  collision_pub_.publish(msg);
}
}  // namespace ros_backtracker
