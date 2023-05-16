#include "commons/trajectory.h"
#include <iostream>
#include <numeric>

namespace crs_controls
{

/**
 * @brief Returns the euclidian distance
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return double distance
 */
inline double euclidean2Dist(double x1, double y1, double x2, double y2)
{
  return (std::sqrt(std::pow(x1 - x2, 2.0) + std::pow(y1 - y2, 2.0)));
}

Eigen::Vector2d Trajectory::getClosestTrackPoint(const Eigen::Vector2d& query_point, int precision /* Default 16 */)
{
  int point_idx = getClosestTrackPointIdx(query_point, precision);

  return Eigen::Vector2d(x_coords_[point_idx], y_coords_[point_idx]);
}

int Trajectory::getClosestTrackPointIdx(const Eigen::Vector2d& query_point, int precision /* Default 16 */)
{
  int num_points = x_coords_.size();
  if (num_points <= 1)
  {
    lastTrackIdx_ = 0;
    return 0;
  }

  int scan_length = 0;
  int best_length;
  double scan_distance;
  double best_distance = 1000;

  // rough search
  for (int scan_length = 0; scan_length < num_points; scan_length += precision)
  {
    scan_distance = euclidean2Dist(query_point.x(), query_point.y(), x_coords_[scan_length], y_coords_[scan_length]);
    if (scan_distance < best_distance)
    {
      best_length = scan_length;
      best_distance = scan_distance;
    }
  }

  //  binary search
  precision = precision / 2;
  int prev_length;
  double prev_distance;
  int next_length;
  double next_distance;
  while (precision >= 0.5)
  {
    prev_length = best_length - precision;
    next_length = best_length + precision;
    if (prev_length >= 0)
    {
      prev_distance = euclidean2Dist(query_point.x(), query_point.y(), x_coords_[prev_length], y_coords_[prev_length]);
      if (prev_distance < best_distance)
      {
        best_length = prev_length;
        best_distance = prev_distance;
      }
    }
    if (next_length <= num_points)
    {
      next_distance = euclidean2Dist(query_point.x(), query_point.y(), x_coords_[next_length], y_coords_[next_length]);
      if (next_distance < best_distance)
      {
        best_length = next_length;
        best_distance = next_distance;
      }
    }
    precision = precision / 2;
  }
  lastTrackIdx_ = best_length;
  return best_length;
}

Eigen::Vector2d Trajectory::operator[](int i) const
{
  // Handle roll over (-1 -> x_coords.size() - 1)
  while (i < 0)
    i += x_coords_.size();

  // Handle roll over (x_coords.size() + 1 -> 1)
  while (i >= x_coords_.size())
    i -= x_coords_.size();

  return Eigen::Vector2d(x_coords_[i], y_coords_[i]);
}

const Eigen::Vector2d Trajectory::getLastRequestedTrackPoint() const
{
  return Eigen::Vector2d(x_coords_[lastTrackIdx_], y_coords_[lastTrackIdx_]);
}

std::vector<double> Trajectory::getVorEdgesX()
{
  return voronoi_edges_x_;
}

std::vector<double> Trajectory::getVorEdgesY()
{
  return voronoi_edges_y_;
}

}  // namespace crs_controls
