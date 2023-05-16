#include <ros/ros.h>

#include <string>
#include "ros_estimators/state_estimator_ros.h"
#include "ros_estimators/component_registry.h"

ros_estimators::RosStateEstimator* estimator = nullptr;

/**
 * @brief Publish estimated state
 *
 * @param event
 */
void publishEstimates(const ros::TimerEvent& event)
{
  estimator->publishState();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "discrete_ekf_node");
  ros::NodeHandle nh = ros::NodeHandle();             // points to /
  ros::NodeHandle nh_private = ros::NodeHandle("~");  //-> points to /estimation_node

  std::string filter_type;

  if (!nh_private.getParam("type", filter_type))
  {
    ROS_ERROR_STREAM("Could not load filter type from parameters. Aborting Estimator!");
    return 1;
  }

  std::string state_type;
  if (!nh_private.getParam("initial_state/type", state_type))
  {
    ROS_ERROR_STREAM("Could not load state filter from parameters. Aborting Estimator!");
    return 1;
  }

  std::string input_type;
  nh_private.getParam("initial_input/type", input_type);

  estimator = ros_estimators::resolveEstimator(nh, nh_private, state_type, input_type, filter_type);

  if (!estimator)
  {
    ROS_ERROR_STREAM("Could not find estimator for given types: (State, Input, Estimator): "
                     << state_type << " " << input_type << " " << filter_type);
    return 1;
  }

  double pub_rate = 100.0;

  if (!nh_private.getParam("pub_rate", pub_rate))
  {
    ROS_WARN_STREAM("Estimator rate not specified. Defaulting to : " << pub_rate);
  }

  ros::Timer timer = nh.createTimer(ros::Duration(1 / pub_rate), publishEstimates);

  ros::MultiThreadedSpinner spinner(1);  // Use 1 Thread
  spinner.spin();                        // spin() will not return until the node has been shutdown

  return 0;
}
