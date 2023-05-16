#ifndef SRC_ROS_ROS_PLANNERS_INCLUDE_ROS_PLANNERS_ROS_PLANNER
#define SRC_ROS_ROS_PLANNERS_INCLUDE_ROS_PLANNERS_ROS_PLANNER
#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>  // ugly, maybe use different message type. Create CRS type
#include <planning/base_planner.h>

#include <ros_crs_utils/state_message_conversion.h>
#include <ros_crs_utils/trajectory_message_conversion.h>
#include <planning/cartesian_reference_point.h>
#include <std_msgs/Bool.h>

namespace ros_planner
{

/**
 * @brief Ros wrapper around a BaseRosController.
 * Note, this must stay a header file in order to have dynamic template support
 *
 */
template <typename StateMsg, typename StateType, typename TrajectoryType>
class RosPlanner
{
private:
  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publisher
  ros::Publisher trajectory_publisher_;

  // Subscriptions
  ros::Subscriber state_subscriber_;
  ros::Subscriber trajectory_finisher_sub_;

  // Last reference trajectory
  std::vector<TrajectoryType> trajectory_;

  std::shared_ptr<crs_planning::BasePlanner<TrajectoryType, StateType>> planner_;

  bool trajectory_finished_ = true;

  /**
   * @brief Max callback rate for this controller
   *
   */
  double max_rate;
  double last_callback = 0.0;

public:
  /**
   * @brief Construct a new Ros Controller object
   *
   * @param nh nodehandle point to ~
   * @param nh_private nodehandle pointing to ~/ros_controller
   * @param planner ptr to the underlying base plannner that this ros wrappers wraps
   */
  RosPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private,
             std::shared_ptr<crs_planning::BasePlanner<TrajectoryType, StateType>> planner)
    : nh_(nh), nh_private_(nh_private), planner_(planner)
  {
    state_subscriber_ = nh_private_.subscribe("state", 10, &RosPlanner::stateCallback, this);
    trajectory_finisher_sub_ =
        nh_private_.subscribe("trajectory_finished", 10, &RosPlanner::trajectoryFinshedCallback, this);
    trajectory_publisher_ = nh_private_.advertise<trajectory_msgs::JointTrajectory>("trajectory", 10);

    if (!nh_private_.getParam("max_rate", max_rate))
    {
      ROS_WARN("[RosController] No Parameter set for max_rate. Will default to 100");
      max_rate = 100.0;
    }
  }

  // This callback can be used to overwrite reached msgs
  void trajectoryFinshedCallback(std_msgs::Bool trajectory_finished)
  {
    trajectory_finished_ = trajectory_finished.data;
  }

  /**
   * @brief State callback, converts the state message to crs state message and calls the controller
   * Note, this callback is rate limit defined by the max_rate parameters
   *
   * @param state_msg the input state message (ros format)
   */
  void stateCallback(StateMsg state_msg)
  {
    // Check rate not exceeded
    if (ros::Time::now().toSec() - last_callback < 1 / max_rate)
      return;

    // Check there is at least one subscriber for trajectory!
    if (trajectory_publisher_.getNumSubscribers() == 0)
      return;

    last_callback = ros::Time::now().toSec();
    auto msg_as_crs_state = message_conversion::convertMsgToState<StateType, StateMsg>(state_msg);
    if (!trajectory_finished_ && !planner_->goalReached(msg_as_crs_state, trajectory_))
      return;

    trajectory_ = planner_->getPlannedTrajectory(msg_as_crs_state);
    trajectory_publisher_.publish(message_conversion::convertToRosTrajectory(trajectory_));  // TODO CONVERT
    trajectory_finished_ = false;
  }
};
}  // namespace ros_planner
#endif /* SRC_ROS_ROS_PLANNERS_INCLUDE_ROS_PLANNERS_ROS_PLANNER */
