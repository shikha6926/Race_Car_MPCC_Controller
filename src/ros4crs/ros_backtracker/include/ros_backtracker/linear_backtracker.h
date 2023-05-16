#ifndef ROS_BACKTRACKER_LINEAR_BACKTRACKER_H
#define ROS_BACKTRACKER_LINEAR_BACKTRACKER_H

#include <crs_msgs/car_input.h>
#include <crs_msgs/car_state_cart.h>
#include <numeric>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace ros_backtracker
{

enum ErrorState
{
  NONE,
  DETECTED,
  CONFIRMED,
  RESIGNED
};

class LinearBackTracker
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher input_pub_;
  ros::Subscriber collision_sub_;
  ros::Subscriber state_sub_;

  // How often to publish an emergency input message
  double publish_freq_ = 20;  // Hz
  // How far to drive back
  double backtrack_distance_ = 0.1;  // m
  // Backtrack command to apply once collided
  double input_torque_ = 0.3;
  // How long to publish backtrack messages before giving up
  double max_backtrack_time_ = 2;

  ErrorState error_state = ErrorState::NONE;
  // Internal use. Saves last collision point
  crs_msgs::car_state_cart reference_point_;

  /**
   * @brief Subscribes to car error state.
   *
   * @param error
   */
  void errorCallback(std_msgs::Bool::ConstPtr error);

  /**
   * @brief State callback. Check if we driven back long enough and stops backtracker
   *
   * @param measured_state
   */
  void stateCallback(crs_msgs::car_state_cart::ConstPtr measured_state);

public:
  /**
   * @brief Construct a new Linear Back Tracker object. Subscribes to error state and car state and starts publishing
   * backtrack messages once the car is stuck / crashed
   *
   * @param nh
   * @param nh_private
   */
  LinearBackTracker(ros::NodeHandle nh, ros::NodeHandle nh_private);
};
}  // namespace ros_backtracker

#endif  // ROS_BACKTRACKER_LINEAR_BACKTRACKER_H
