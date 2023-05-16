#ifndef ROS_BACKTRACKER_INPUT_FILTER_H
#define ROS_BACKTRACKER_INPUT_FILTER_H

#include <crs_msgs/car_input.h>
#include <ros/ros.h>

namespace ros_backtracker
{
class InputFilter
{
private:
  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publisher
  ros::Publisher input_publisher_;

  // Subscriptions
  ros::Subscriber regular_input_sub_;
  // Subscriptions
  ros::Subscriber emergency_input_sub_;

  // How long to wait after the last emergency input to switch back to the main controller input
  double block_time_ = 0.1;
  double emergency_mode_ = false;
  double last_emergency_input_ts_ = 0;

  /**
   * @brief
   *
   * @param input_msg
   */
  void inputCallback(crs_msgs::car_input::ConstPtr input_msg);
  /**
   * @brief
   *
   * @param input_msg
   */
  void emergencyInputCallback(crs_msgs::car_input::ConstPtr input_msg);

public:
  /**
   * @brief Construct a new Input Filter object. Subscribes to input and emergencyInput topics and selectively publishes
   * only on of them depending on the collision state
   *
   * @param nh
   * @param nh_private
   */
  InputFilter(ros::NodeHandle nh, ros::NodeHandle nh_private);
};
}  // namespace ros_backtracker

#endif
