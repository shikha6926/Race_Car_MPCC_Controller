#ifndef ROS_CONTROLLERS_ROS_CONTROLLER_H
#define ROS_CONTROLLERS_ROS_CONTROLLER_H
#include <ros/ros.h>

#include <controls/base_controller.h>
#include <ros_crs_utils/state_message_conversion.h>

#include <visualization_msgs/Marker.h>

#include "ros_controllers/visualizers/base_visualizer.h"

namespace ros_controllers
{

/**
 * @brief Ros wrapper around a BaseRosController.
 * Note, this must stay a header file in order to have dynamic template support
 *
 */
template <typename StateMsg, typename InputMsg, typename StateType, typename InputType>
class RosController
{
private:
  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publisher
  ros::Publisher input_publisher_;

  ros::Timer visualization_timer_;

  // Subscriptions
  ros::Subscriber state_subscriber_;

  // Visualizer for controller
  std::unique_ptr<BaseControllerVisualizer<StateType, InputType>> visualizer_;

  /**
   * @brief Max callback rate for this controller
   *
   */
  double max_rate;
  double last_callback = 0.0;

public:
  /**
   * TODO UGLY, move back to private, maybe use getter() ore expose differently
   *
   * @brief Reference to base controller
   *
   */
  std::shared_ptr<crs_controls::BaseController<StateType, InputType>> controller_;

  /**
   * @brief Construct a new Ros Controller object
   *
   * @param nh nodehandle point to ~
   * @param nh_private nodehandle pointing to ~/ros_controller
   * @param controller ptr to the underlying base controller that this ros wrappers wraps
   */
  RosController(ros::NodeHandle nh, ros::NodeHandle nh_private,
                std::unique_ptr<BaseControllerVisualizer<StateType, InputType>> visualizer,
                std::shared_ptr<crs_controls::BaseController<StateType, InputType>> controller)
    : nh_(nh), nh_private_(nh_private), controller_(controller)
  {
    input_publisher_ = nh_private_.advertise<InputMsg>("control_input", 10);
    state_subscriber_ = nh_private_.subscribe("state", 10, &RosController::stateCallback, this);

    if (!nh_private_.getParam("max_rate", max_rate))
    {
      ROS_WARN("[RosController] No Parameter set for max_rate. Will default to 100");
      max_rate = 100.0;
    }
    visualizer_ = std::move(visualizer);
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
    if (ros::Time::now().toSec() - last_callback <= 1 / max_rate)
      return;

    if (controller_->isInitializing())
      return;

    last_callback = ros::Time::now().toSec();
    InputType input = controller_->getControlInput(
        message_conversion::convertMsgToState<StateType, StateMsg>(state_msg), last_callback);
    input_publisher_.publish(message_conversion::convertToRosInput<InputMsg, InputType>(input));
  }
};
}  // namespace ros_controllers
#endif
