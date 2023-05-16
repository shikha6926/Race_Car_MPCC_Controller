#ifndef ROS_CONTROLLERS_VISUALIZERS_LAST_REFERENCE_POINT_VISUALIZER_H
#define ROS_CONTROLLERS_VISUALIZERS_LAST_REFERENCE_POINT_VISUALIZER_H

#include "ros_controllers/visualizers/base_visualizer.h"
#include <controls/base_controller.h>
#include <ros/ros.h>
#include <ros_crs_utils/state_message_conversion.h>
#include <visualization_msgs/Marker.h>

namespace ros_controllers
{

template <typename StateType, typename InputType>
class LastReferencePointVisualizer : public BaseControllerVisualizer<StateType, InputType>
{
private:
  // Configurable parameters
  std::string frame_id = "crs_frame";
  std::string ns = "controller";
  double size_x = 0.05;
  double size_y = 0.05;
  double size_z = 0.05;

  double color_r = 0;
  double color_g = 0;
  double color_b = 0;
  double color_a = 1;
  bool use_arrows_ = false;
  visualization_msgs::Marker marker;

public:
  LastReferencePointVisualizer(ros::NodeHandle nh_private,
                               std::shared_ptr<crs_controls::BaseController<StateType, InputType>> controller)
    : BaseControllerVisualizer<StateType, InputType>(nh_private, controller)
  {
    // Load Params
    nh_private.getParam("frame_id", frame_id);
    nh_private.getParam("namespace", ns);

    nh_private.getParam("size_x", size_x);
    nh_private.getParam("size_y", size_y);
    nh_private.getParam("size_z", size_z);

    nh_private.getParam("r", color_r);
    nh_private.getParam("g", color_g);
    nh_private.getParam("b", color_b);
    nh_private.getParam("a", color_a);

    nh_private.getParam("use_arrows", use_arrows_);

    // Setup Marker
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = use_arrows_ ? visualization_msgs::Marker::ARROW : visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    // Scales
    marker.scale.x = size_x;
    marker.scale.y = size_y;
    marker.scale.z = size_z;
    // Color
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    marker.color.a = color_a;
  };

  void visualizationCallback(const ros::TimerEvent& event) override
  {
    auto lastPoint =
        BaseControllerVisualizer<StateType, InputType>::controller_->getTrajectory()->getLastRequestedTrackPoint();

    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = lastPoint.x();
    marker.pose.position.y = lastPoint.y();

    if (use_arrows_)
    {
      double yaw =
          BaseControllerVisualizer<StateType, InputType>::controller_->getTrajectory()->getLastRequestedTrackAngle();

      // Planned orientation (quaternion from only yaw)
      marker.pose.orientation.w = std::cos(yaw * 0.5);
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = std::sin(yaw * 0.5);
    }
    BaseControllerVisualizer<StateType, InputType>::visualization_publisher_.publish(marker);
  }
};

};  // namespace ros_controllers

#endif  // ROS_CONTROLLERS_VISUALIZERS_LAST_REFERENCE_POINT_VISUALIZER_H
