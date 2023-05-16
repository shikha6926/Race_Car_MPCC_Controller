#ifndef ROS_BACKTRACKER_CRASH_DETECTOR_H
#define ROS_BACKTRACKER_CRASH_DETECTOR_H

#include <commons/static_track_trajectory.h>
#include <crs_msgs/car_state_cart.h>
#include <numeric>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace ros_backtracker
{
class CrashDetector
{
private:
  std::shared_ptr<crs_controls::StaticTrackTrajectory> static_track_trajectory_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // How much slack we give on the track
  double distance_tolerance_ = 0.0;
  // How long (s) the car has to not be moving to trigger the crash detector
  double time_tolerance_ = 1.0;
  // How much the car must have moved during 'time_tolerance' to trigger the crash detector
  double min_move_distance_ = 0.05;
  // How often we check for collisision
  double check_rate_ = 5;  // Hz;

  // Needed to keep track of msg rate
  double last_message_ts_ = 0.0;

  // send collision messages at 5hz
  double collision_msg_rate_ = 5;
  // Needed to keep track of msg rate
  double last_collision_msg_ts_ = 0.0;
  // Internal state
  bool collision_detected_ = false;

  // Publishes collision messages
  ros::Publisher collision_pub_;
  // State subscriber
  ros::Subscriber state_sub_;

  // Queue to save last states to check if car moved during certain time
  std::deque<crs_msgs::car_state_cart> last_states_;

  /**
   * @brief  Loads node specific parameters
   *
   */
  void loadParams();

  /**
   * @brief State callback, checks if collision occurred and if yes published to 'collision_pub_'
   *
   * @param measured_state
   */
  void callback(crs_msgs::car_state_cart::ConstPtr measured_state);

public:
  /**
   * @brief Construct a new Crash Detector object
   *
   * @param nh public nodehandle
   * @param nh_private  private nodehandle
   * @param static_track_trajectory track manager used to find collisions
   */
  CrashDetector(ros::NodeHandle nh, ros::NodeHandle nh_private,
                std::shared_ptr<crs_controls::StaticTrackTrajectory> static_track_trajectory);
};
}  // namespace ros_backtracker

#endif  // ROS_BACKTRACKER_CRASH_DETECTOR_H
