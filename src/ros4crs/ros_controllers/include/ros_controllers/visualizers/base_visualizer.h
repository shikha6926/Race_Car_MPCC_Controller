#ifndef ROS_CONTROLLERS_VISUALIZERS_BASE_VISUALIZER_H
#define ROS_CONTROLLERS_VISUALIZERS_BASE_VISUALIZER_H

#include <controls/base_controller.h>
#include <ros/ros.h>

namespace ros_controllers
{

/**
 * @brief Visualizer that shows planned control inputs
 *
 * @tparam StateType
 * @tparam InputType
 */
template <typename StateType, typename InputType>
class BaseControllerVisualizer
{
private:
  double visualization_rate = 10;

protected:
  ros::Publisher visualization_publisher_;
  ros::Timer visualization_timer_;
  std::shared_ptr<crs_controls::BaseController<StateType, InputType>> controller_;

public:
  BaseControllerVisualizer(ros::NodeHandle nh_private,
                           std::shared_ptr<crs_controls::BaseController<StateType, InputType>> controller)
    : controller_(controller)
  {
    visualization_publisher_ = nh_private.advertise<visualization_msgs::Marker>("controller_visualization", 100);

    if (!nh_private.getParam("rate", visualization_rate))
    {
      visualization_rate = 10.0;
      ROS_WARN_STREAM("No visualization rate specified for controller visualizer! Checked namespace:"
                      << nh_private.getNamespace() << " Defaulting to " << visualization_rate << " Hz");
    }

    if (visualization_rate != 0.0)
      visualization_timer_ = nh_private.createTimer(ros::Duration(1 / visualization_rate),
                                                    &BaseControllerVisualizer::visualizationCallback, this);
  };

  virtual void visualizationCallback(const ros::TimerEvent& event) = 0;
};
}  // namespace ros_controllers

#endif  // ROS_CONTROLLERS_VISUALIZERS_BASE_VISUALIZER_H
