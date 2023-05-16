#ifndef SRC_ROS_ROS_PLANNERS_INCLUDE_ROS_MULTI_AGENT_PLANNERS_ROS_PLANNER
#define SRC_ROS_ROS_PLANNERS_INCLUDE_ROS_MULTI_AGENT_PLANNERS_ROS_PLANNER
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
class RosMultiAgentPlanner
{
private:
  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::vector<std::string> car_namespaces_;
  // Publisher mapped (namespace to pub)
  std::map<std::string, ros::Publisher> trajectory_publishers_;
  std::map<std::string, ros::Publisher> voredges_publisher_;

  // Subscriptions

  std::vector<ros::Subscriber> state_subscribers_;
  ros::Subscriber trajectory_finisher_sub_;

  // Last reference trajectory
  std::vector<TrajectoryType> trajectory_;

  std::shared_ptr<crs_planning::BasePlanner<TrajectoryType, std::map<std::string, StateType>>> planner_;

  bool trajectory_finished_ = true;

  std::map<std::string, StateMsg> car_to_state_msg_;
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
  RosMultiAgentPlanner(
      ros::NodeHandle nh, ros::NodeHandle nh_private,
      std::shared_ptr<crs_planning::BasePlanner<TrajectoryType, std::map<std::string, StateType>>> planner)
    : nh_(nh), nh_private_(nh_private), planner_(planner)
  {
    if (!nh_private_.getParam("max_rate", max_rate))
    {
      ROS_WARN("[RosMultiAgentPlanner] No Parameter set for max_rate. Will default to 100");
      max_rate = 100.0;
    }

    if (!nh_.getParam("car_namespaces", car_namespaces_))
    {
      ROS_WARN("[RosMultiAgentPlanner] Car Namespaces Missing!");
    }

    for (const std::string ns : car_namespaces_)
    {
      auto sub = nh_.subscribe<StateMsg>(ns + "/estimation_node/best_state", 10,
                                         boost::bind(&RosMultiAgentPlanner::singleCarStateCallback, this, _1, ns));
      state_subscribers_.push_back(sub);

      trajectory_publishers_[ns] = nh_.advertise<trajectory_msgs::JointTrajectory>(ns + "/trajectory", 10);
      std::cout << "creating publisher for  " << ns + "/trajecotry" << std::endl;

      voredges_publisher_[ns] = nh_.advertise<geometry_msgs::PolygonStamped>(ns + "/vor_edges",10);
    }
    trajectory_finisher_sub_ =
        nh_.subscribe("trajectory_finished", 10, &RosMultiAgentPlanner::trajectoryFinshedCallback, this);
  }

  // This callback can be used to overwrite reached msgs
  void trajectoryFinshedCallback(std_msgs::Bool trajectory_finished)
  {
    trajectory_finished_ = trajectory_finished.data;
  }

  void singleCarStateCallback(const boost::shared_ptr<StateMsg const> state_msg, const std::string car_namespace)
  {
    car_to_state_msg_[car_namespace] = *state_msg;
    stateCallback(car_to_state_msg_);
  }

  /**
   * @brief State callback, converts the state message to crs state message and calls the controller
   * Note, this callback is rate limit defined by the max_rate parameters
   *
   * @param state_msg the input state message (ros format)
   */
  void stateCallback(std::map<std::string, StateMsg> state_msg)
  {
    // Check rate not exceeded
    if (ros::Time::now().toSec() - last_callback < 1 / max_rate)
      return;

    last_callback = ros::Time::now().toSec();

    // Check there is at least one subscriber for trajectory!
    for (auto& entry : trajectory_publishers_)
    {
      if (entry.second.getNumSubscribers() == 0)
      {
        return;
      }
    }

    std::map<std::string, StateType> ns_to_crs_state;
    for (auto& entry : state_msg)
    {
      ns_to_crs_state[entry.first] = message_conversion::convertMsgToState<StateType>(entry.second);
    }

    if (!trajectory_finished_ && !planner_->goalReached(ns_to_crs_state, trajectory_))
      return;

    ROS_INFO("try get planed trajectory");
    trajectory_ = planner_->getPlannedTrajectory(ns_to_crs_state);
    ROS_INFO("got planed trajectory");
    std::vector<std::vector<double>> vor_edges_x = planner_->getVorEdgesX();
    std::vector<std::vector<double>> vor_edges_y = planner_->getVorEdgesY();
    int i = 0;
    for (auto ns : car_namespaces_)
    {
      trajectory_publishers_[ns].publish(message_conversion::convertToRosTrajectory(trajectory_, ns));
      voredges_publisher_[ns].publish(message_conversion::convertToRosVoronoi(vor_edges_x[i], vor_edges_y[i]));
      i++;
    }

    trajectory_finished_ = false;
  }
};
}  // namespace ros_planner
#endif /* SRC_ROS_ROS_PLANNERS_INCLUDE_ROS_PLANNERS_ROS_PLANNER */
