#include "ros_crs_utils/trajectory_message_conversion.h"
#include <planning/cartesian_reference_point.h>
#include <planning/multi_car_cartesian_reference_point.h>
#include <vector>

namespace message_conversion
{
// TODO, we are really just hijakcing the joint trajectory format. There must either be a better suited message or we
// should define our own
template <>
trajectory_msgs::JointTrajectory
convertToRosTrajectory(const std::vector<crs_planning::cartesian_reference_point>& trajectory,
                       std::string car_name /* = "" */)
{
  trajectory_msgs::JointTrajectory ros_trajectory;
  ros_trajectory.joint_names.push_back(car_name);
  for (const auto& pt : trajectory)
  {
    trajectory_msgs::JointTrajectoryPoint ros_pt;
    ros_pt.positions.push_back(pt.x);
    ros_pt.positions.push_back(pt.y);
    ros_pt.positions.push_back(0);  // z = 0;
    ros_trajectory.points.push_back(ros_pt);
  }
  return ros_trajectory;
}

template <>
trajectory_msgs::JointTrajectory
convertToRosTrajectory(const std::vector<crs_planning::multi_car_cartesian_reference_point>& trajectory,
                       std::string car_name /* = "" */)
{
  trajectory_msgs::JointTrajectory ros_trajectory;
  ros_trajectory.joint_names.push_back(car_name);

  int car_index = 0;
  if (!trajectory.empty())
  {
    for (car_index = 0; car_index < trajectory[0].namespaces.size(); car_index++)
    {
      if (trajectory[0].namespaces[car_index] == car_name)
        break;
    }
  }

  for (const auto& pt : trajectory)
  {
    trajectory_msgs::JointTrajectoryPoint ros_pt;
    ros_pt.positions.push_back(pt.points[car_index].x);
    ros_pt.positions.push_back(pt.points[car_index].y);
    ros_pt.positions.push_back(0);  // z = 0;
    ros_trajectory.points.push_back(ros_pt);
  }
  return ros_trajectory;
}


geometry_msgs::PolygonStamped
convertToRosVoronoi(const std::vector<double> vor_edges_x, const std::vector<double> vor_edges_y)
{
  geometry_msgs::PolygonStamped poly_msg;
  poly_msg.header.frame_id = "crs_frame";
  for (int i = 0; i < vor_edges_x.size(); i++)
  {
    geometry_msgs::Point32 point;
    point.x = vor_edges_x[i];
    point.y = vor_edges_y[i];
    poly_msg.polygon.points.push_back(point);
  }
  return poly_msg;
}

}  // namespace message_conversion
