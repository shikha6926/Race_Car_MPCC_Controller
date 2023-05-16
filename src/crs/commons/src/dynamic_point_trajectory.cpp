#include "commons/dynamic_point_trajectory.h"
#include <iostream>
#include <numeric>

namespace crs_controls
{
void DynamicPointTrajectory::resetTrajectory(std::vector<double> x_coord, std::vector<double> y_coord)
{
  x_coords_ = x_coord;
  y_coords_ = y_coord;
  lastTrackIdx_ = -1;
}

void DynamicPointTrajectory::resetTrajectory(std::vector<Eigen::Vector2d> pts)
{
  x_coords_.clear();
  y_coords_.clear();
  for (auto& pt : pts)
  {
    x_coords_.push_back(pt.x());
    y_coords_.push_back(pt.y());
  }
  lastTrackIdx_ = -1;
}

void DynamicPointTrajectory::resetVorEdges(std::vector<double> x_edge, std::vector<double> y_edge)
{
  voronoi_edges_x_ = x_edge;
  voronoi_edges_y_ = y_edge;
}

const double DynamicPointTrajectory::getLastRequestedTrackAngle() const
{
  return 0;
}
}  // namespace crs_controls
