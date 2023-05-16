#ifndef SRC_ROS_ROS_CONTROLLERS_INCLUDE_ROS_CONTROLLERS_VISUALIZERS_MPC_CONTROLLER_VISUALIZER
#define SRC_ROS_ROS_CONTROLLERS_INCLUDE_ROS_CONTROLLERS_VISUALIZERS_MPC_CONTROLLER_VISUALIZER
#include <ros/ros.h>

#include <controls/base_controller.h>
#include <ros_crs_utils/state_message_conversion.h>

#include "ros_controllers/visualizers/base_visualizer.h"
#include <controls/mpc_controller.h>
#include <visualization_msgs/Marker.h>

namespace ros_controllers
{

template <typename ModelType, typename StateType, typename InputType>
class MpcControllerVisualizer : public BaseControllerVisualizer<StateType, InputType>
{
private:
  // Use arrow to also show planned and reference yaw angle
  bool use_arrows = false;
  // Configurable parameters
  std::string frame_id = "crs_frame";
  double planned_size_x = 0.05;
  double planned_size_y = 0.05;
  double planned_size_z = 0.05;

  double ref_size_x = 0.05;
  double ref_size_y = 0.05;
  double ref_size_z = 0.05;

  double ref_color_r = 0;
  double ref_color_g = 0;
  double ref_color_b = 0;
  double ref_color_a = 0;

  double min_velocity = 1.5;
  double max_velocity = 2.5;

  std::string ns = "";

  visualization_msgs::Marker reference;
  visualization_msgs::Marker planned;

public:
  MpcControllerVisualizer(ros::NodeHandle nh_private,
                          std::shared_ptr<crs_controls::MpcController<ModelType, StateType, InputType>> controller)
    : BaseControllerVisualizer<StateType, InputType>(
          nh_private, std::dynamic_pointer_cast<crs_controls::BaseController<StateType, InputType>>(controller))
  {
    nh_private.getParam("use_arrows", use_arrows);

    nh_private.getParam("frame_id", frame_id);

    nh_private.getParam("min_velocity", min_velocity);
    nh_private.getParam("max_velocity", max_velocity);

    nh_private.getParam("planned/size_x", planned_size_x);
    nh_private.getParam("planned/size_y", planned_size_y);
    nh_private.getParam("planned/size_z", planned_size_z);

    nh_private.getParam("reference/r", ref_color_r);
    nh_private.getParam("reference/g", ref_color_g);
    nh_private.getParam("reference/b", ref_color_b);
    nh_private.getParam("reference/a", ref_color_a);
    nh_private.getParam("reference/size_x", ref_size_x);
    nh_private.getParam("reference/size_y", ref_size_y);
    nh_private.getParam("reference/size_z", ref_size_z);

    nh_private.getParam("namespace", ns);

    // Reference on track
    reference.header.frame_id = frame_id;
    reference.ns = ns + "_reference_trajectory";
    reference.id = 1;
    reference.scale.x = planned_size_x;
    reference.scale.y = planned_size_y;
    reference.scale.z = planned_size_z;
    // Reference has static color as there is no target velocity on the reference
    reference.color.r = ref_color_r;
    reference.color.g = ref_color_g;
    reference.color.b = ref_color_b;
    reference.color.a = ref_color_a;
    reference.action = visualization_msgs::Marker::ADD;
    reference.type = use_arrows ? visualization_msgs::Marker::ARROW : visualization_msgs::Marker::POINTS;

    // Planned trajectory
    planned.header.frame_id = frame_id;
    planned.ns = ns + "_planned_trajectory";
    planned.id = 0;
    planned.scale.x = planned_size_x;
    planned.scale.y = planned_size_y;
    planned.scale.z = planned_size_z;
    planned.action = visualization_msgs::Marker::ADD;
    planned.type = use_arrows ? visualization_msgs::Marker::ARROW : visualization_msgs::Marker::POINTS;
  };

  void visualizationCallback(const ros::TimerEvent& event) override
  {
    reference.header.stamp = ros::Time::now();
    planned.header.stamp = ros::Time::now();
    planned.points.clear();
    planned.colors.clear();
    reference.points.clear();

    geometry_msgs::Point temp_point;

    auto mpc_controller_ptr = std::dynamic_pointer_cast<crs_controls::MpcController<ModelType, StateType, InputType>>(
        BaseControllerVisualizer<StateType, InputType>::controller_);

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
      if (!use_arrows)
      {
        // Position
        temp_point.x = trajectory[i][0];  // x_position
        temp_point.y = trajectory[i][1];  // y_position
        temp_point.z = 0;
        planned.points.push_back(temp_point);
        planned.colors.push_back(color);
        // Refrence
        temp_point.x = trajectory[i][4];  // x_position on track
        temp_point.y = trajectory[i][5];  // y_position on track
        temp_point.z = 0.01;
        reference.points.push_back(temp_point);
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

        // Reference on track
        // Reference position
        reference.pose.position.x = trajectory[i][4];
        reference.pose.position.y = trajectory[i][5];
        reference.pose.position.z = 0;

        // Reference orientation (quaternion from only yaw)
        reference.pose.orientation.w = std::cos(trajectory[i][6] * 0.5);
        reference.pose.orientation.x = 0;
        reference.pose.orientation.y = 0;
        reference.pose.orientation.z = std::sin(trajectory[i][6] * 0.5);

        reference.id = id_cnter++;

        BaseControllerVisualizer<StateType, InputType>::visualization_publisher_.publish(reference);
        BaseControllerVisualizer<StateType, InputType>::visualization_publisher_.publish(planned);
      }
    }

    if (!use_arrows)
    {
      // If we do not use arrows, only need to publish one marker containing all points.
      BaseControllerVisualizer<StateType, InputType>::visualization_publisher_.publish(reference);
      BaseControllerVisualizer<StateType, InputType>::visualization_publisher_.publish(planned);
    }
  }
};

};  // namespace ros_controllers

#endif /* SRC_ROS_ROS_CONTROLLERS_INCLUDE_ROS_CONTROLLERS_VISUALIZERS_MPC_CONTROLLER_VISUALIZER */
