#include "ros_controllers/trajectory/dynamic_trajectory_updater.h"
#include <vector>
namespace ros_controllers
{
DynamicTrajectoryUpdator::DynamicTrajectoryUpdator(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                                   std::shared_ptr<crs_controls::DynamicPointTrajectory> trajectory)
  : nh_(nh), nh_private_(nh_private), trajectory_(trajectory)
{
  traj_sub_ = nh_.subscribe("trajectory", 10, &DynamicTrajectoryUpdator::rosTrajectoryCallback, this);
  vor_edges_sub_ = nh_.subscribe("vor_edges", 10, &DynamicTrajectoryUpdator::rosVorEdgesCallback, this);
  std::cout << "created DynamicTrajectoryUpdator" << std::endl;
  std::cout << "anemsapce" << nh_.getNamespace() << std::endl;
}

void DynamicTrajectoryUpdator::rosTrajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr msg)
{
  std::cout << "got new trajectory in namespace: " << nh_.getNamespace() << " resetting!" << std::endl;
  std::vector<double> x_coords;
  std::vector<double> y_coords;

  for (const auto& pt : msg->points)
  {
    // Ugly!
    double x = pt.positions[0];
    double y = pt.positions[1];

    x_coords.push_back(x);
    y_coords.push_back(y);
  }

  trajectory_->resetTrajectory(x_coords, y_coords);
  std::cout << "Updated trajectory!" << std::endl;
}

void DynamicTrajectoryUpdator::rosVorEdgesCallback(const geometry_msgs::PolygonStampedPtr msg)
{
  std::cout << "got new voronoi partition namespace: " << nh_.getNamespace() << " resetting!" << std::endl;
  std::vector<double> x_coords;
  std::vector<double> y_coords;

  for (const auto& pt : msg->polygon.points)
  {
    // Ugly!
    double x = pt.x;
    double y = pt.y;

    x_coords.push_back(x);
    y_coords.push_back(y);
  }

  trajectory_->resetVorEdges(x_coords, y_coords);
  std::cout << "Updated Voronoi!" << std::endl;
}

}  // namespace ros_controllers
