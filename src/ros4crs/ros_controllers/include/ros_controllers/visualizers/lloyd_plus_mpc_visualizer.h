#ifndef ROS_CONTROLLERS_VISUALIZERS_LLOYD_PLUS_MPC_VISUALIZER_H
#define ROS_CONTROLLERS_VISUALIZERS_LLOYD_PLUS_MPC_VISUALIZER_H

#include "ros_controllers/visualizers/base_visualizer.h"
#include <controls/base_controller.h>
#include <controls/mpc_controller.h>
#include <ros/ros.h>
#include <ros_crs_utils/state_message_conversion.h>
#include <visualization_msgs/Marker.h>

namespace ros_controllers
{

template <typename ModelType, typename StateType, typename InputType>
class LloydPlusMPCVisualizer : public BaseControllerVisualizer<StateType, InputType>
{
private:
  // Configurable parameters
  std::string frame_id = "crs_frame";
  std::string ns = "";
  double size_x = 0.05;
  double size_y = 0.05;
  double size_z = 0.05;

  double color_r = 0;
  double color_g = 0;
  double color_b = 0;
  double color_a = 1;

  double planned_size_x = 0.05;
  double planned_size_y = 0.05;
  double planned_size_z = 0.05;

  double min_velocity = 1.5;
  double max_velocity = 2.5;

  // Use arrow to also show planned and reference yaw angle
  bool use_arrows_ = false;

  // Define marker
  visualization_msgs::Marker marker;
  visualization_msgs::Marker line;
  visualization_msgs::Marker planned;

public:
  LloydPlusMPCVisualizer(ros::NodeHandle nh_private,
                          std::shared_ptr<crs_controls::MpcController<ModelType, StateType, InputType>> controller)
    : BaseControllerVisualizer<StateType, InputType>(
          nh_private, std::dynamic_pointer_cast<crs_controls::BaseController<StateType, InputType>>(controller))
    {
    // Load Params
    nh_private.getParam("frame_id", frame_id);
    nh_private.getParam("namespace", ns);

    nh_private.getParam("size_x", size_x);
    nh_private.getParam("size_y", size_y);
    nh_private.getParam("size_z", size_z);

    nh_private.getParam("min_velocity", min_velocity);
    nh_private.getParam("max_velocity", max_velocity);

    nh_private.getParam("planned/size_x", planned_size_x);
    nh_private.getParam("planned/size_y", planned_size_y);
    nh_private.getParam("planned/size_z", planned_size_z);

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

    // Setup line
    line.header.frame_id = frame_id;
    line.header.stamp = ros::Time::now();
    line.ns = ns;
    line.id = 1;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.1;
    line.color.g = 1.0;
    line.color.a = 1.0;

    // Planned trajectory
    planned.header.frame_id = frame_id;
    planned.ns = ns + "_planned_trajectory";
    planned.id = 2;
    planned.scale.x = planned_size_x;
    planned.scale.y = planned_size_y;
    planned.scale.z = planned_size_z;
    planned.action = visualization_msgs::Marker::ADD;
    planned.type = use_arrows_ ? visualization_msgs::Marker::ARROW : visualization_msgs::Marker::POINTS;
  };

  void visualizationCallback(const ros::TimerEvent& event) override
  {
    auto mpc_controller_ptr = std::dynamic_pointer_cast<crs_controls::MpcController<ModelType, StateType, InputType>>(
        BaseControllerVisualizer<StateType, InputType>::controller_);

    auto lastPoint = mpc_controller_ptr->getTrajectory()->getLastRequestedTrackPoint();

    marker.header.stamp = ros::Time::now();
    line.header.stamp = ros::Time::now();
    planned.header.stamp = ros::Time::now();
    marker.pose.position.x = lastPoint.x();
    marker.pose.position.y = lastPoint.y();

    line.points.clear();
    planned.points.clear();
    planned.colors.clear();
    geometry_msgs::Point temp_point;


    std::vector<double> vor_edges_x =
        mpc_controller_ptr->getTrajectory()->getVorEdgesX();

    std::vector<double> vor_edges_y =
        mpc_controller_ptr->getTrajectory()->getVorEdgesY();

    geometry_msgs::Point p;

    for (int i=0; i<vor_edges_x.size(); i++)
    {
      p.x = vor_edges_x[i];
      p.y = vor_edges_y[i];
      line.points.push_back(p);
      if(i == (vor_edges_x.size()-1))
      {
        p.x = vor_edges_x[0];
        p.y = vor_edges_y[0];
        line.points.push_back(p);
      }
    }

    int id_cnter = 0;  // Creates unique IDs for the markers. Only used in arrow mode
    std::vector<std::vector<double>> trajectory = mpc_controller_ptr->getPlannedTrajectory();
    for (int i = 0; i < trajectory.size(); i++)
    {
      // Create color gradient from blue (lowest) to red (highest) depending on velocity
      std_msgs::ColorRGBA color;
      double normalized_velocity =
          std::max(0.0, std::min(1.0, (trajectory[i][2] - min_velocity) / (max_velocity - min_velocity)));
      color.r = normalized_velocity < 0.5 ? normalized_velocity : 1 - normalized_velocity;
      color.g = normalized_velocity < 0.5 ? 0 : normalized_velocity;
      color.b = normalized_velocity < 0.5 ? 1 - normalized_velocity : 0;
      color.a = 1;

      // ========= Do not use arrows (-> no yaw visualization)
      if (!use_arrows_)
      {
        // Position
        temp_point.x = trajectory[i][0];  // x_position
        temp_point.y = trajectory[i][1];  // y_position
        temp_point.z = 0;
        planned.points.push_back(temp_point);
        planned.colors.push_back(color);
      }
      else
      {
        // Planned position
        planned.pose.position.x = trajectory[i][0];
        planned.pose.position.y = trajectory[i][1];
        planned.pose.position.z = 0;
        // Planned orientation (quaternion from only yaw)
        planned.pose.orientation.w = std::cos(trajectory[i][3] * 0.5);
        planned.pose.orientation.x = 0;
        planned.pose.orientation.y = 0;
        planned.pose.orientation.z = std::sin(trajectory[i][3] * 0.5);

        planned.color.r = color.r;
        planned.color.g = color.g;
        planned.color.b = color.b;
        planned.color.a = color.a;

        planned.id = id_cnter++;

        BaseControllerVisualizer<StateType, InputType>::visualization_publisher_.publish(planned);
      }
    }

    if (!use_arrows_)
    {
      // If we do not use arrows, only need to publish one marker containing all points.
      BaseControllerVisualizer<StateType, InputType>::visualization_publisher_.publish(planned);
    }

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
    BaseControllerVisualizer<StateType, InputType>::visualization_publisher_.publish(line);
  }
};

};  // namespace ros_controllers

#endif  // ROS_CONTROLLERS_VISUALIZERS_LAST_REFERENCE_POINT_VISUALIZER_H
