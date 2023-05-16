#include "commons/static_track_trajectory.h"
#include <iostream>
#include <numeric>

namespace crs_controls
{

void StaticTrackTrajectory::unwrapYawAngle()
{
  for (int i = 0; i < tangent_angle_.size(); i++)
  {
    tangent_angle_unwrapped_[i] = tangent_angle_[i];
    double tan_ang = tangent_angle_[i];
    if (i == 0)
      tangent_angle_unwrapped_[i] = tan_ang;
    else
    {
      double diff = tan_ang - tangent_angle_unwrapped_[i - 1];
      if (abs(diff) < 1.0)
        tangent_angle_unwrapped_[i] = tan_ang;
      else
        tangent_angle_unwrapped_[i] = M_PI + (M_PI + tan_ang);
    }
  }

  for (int i = tangent_angle_.size(); i < tangent_angle_unwrapped_.size(); i++)
  {
    tangent_angle_unwrapped_[i] = tangent_angle_unwrapped_[i - tangent_angle_.size()] + 2.0 * M_PI;
  }
}

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

const double StaticTrackTrajectory::getLastRequestedTrackAngle() const
{
  return getTrackAngle(lastTrackIdx_);
}

StaticTrackTrajectory::StaticTrackTrajectory(const std::vector<Eigen::Vector2d>& points) : Trajectory()
{
  x_coords_.reserve(points.size());
  y_coords_.reserve(points.size());
  for (const auto& pt : points)
  {
    x_coords_.push_back(pt.x());
    y_coords_.push_back(pt.y());
  }
}
const double StaticTrackTrajectory::getMeanCurvatureAlongPath(int start_idx, int distance) const
{
  std::vector<double> abs_curv(distance, 0.0);
  std::transform(curvature_.begin() + start_idx, curvature_.begin() + start_idx + distance, abs_curv.begin(),
                 std::abs<double>);
  double sum = std::accumulate(abs_curv.begin(), abs_curv.end(), 0.0);
  return sum / distance;
}

std::vector<Eigen::Vector2d> StaticTrackTrajectory::getCenterLine() const
{
  std::vector<Eigen::Vector2d> center_line;
  center_line.reserve(x_coords_.size());
  for (int i = 0; i < x_coords_.size(); i++)
  {
    center_line.push_back(Eigen::Vector2d(x_coords_[i], y_coords_[i]));
  }
  return center_line;
}

const double StaticTrackTrajectory::getCurvature(int i) const
{
  return curvature_[i % curvature_.size()];
}
const double StaticTrackTrajectory::getTrackAngle(int i) const
{
  return tangent_angle_unwrapped_[i % tangent_angle_unwrapped_.size()];
}

track_error StaticTrackTrajectory::getTrackError(const Eigen::Vector2d& query_point, int precision /* default 16 */)
{
  int track_idx = getClosestTrackPointIdx(query_point, precision);

  const auto& car_pt = query_point;
  const Eigen::Vector2d& closest_pt = this->operator[](track_idx);
  const Eigen::Vector2d& next_pt = this->operator[](track_idx + 1);  // Operator Handles roll over

  // Calculate Side
  // -1 if car on the left of the track, +1 if car on the right of track
  int side = (((next_pt.x() - closest_pt.x()) * (car_pt.y() - closest_pt.y()) -
               (next_pt.y() - closest_pt.y()) * (car_pt.x() - closest_pt.x())) > 0) ?
                 -1 :
                 1;

  // Calculate lateral error
  double err_num = std::abs((next_pt.y() - closest_pt.y()) * car_pt.x() - (next_pt.x() - closest_pt.x()) * car_pt.y() +
                            next_pt.x() * closest_pt.y() - next_pt.y() * closest_pt.x());
  double err_den = euclidean2Dist(closest_pt.x(), closest_pt.y(), next_pt.x(), next_pt.y());
  double lat_error = err_num / err_den;
  return { track_idx, side, lat_error };
}

void StaticTrackTrajectory::changeTangentAngle(int direction)
{
  for (int i = 0; i < tangent_angle_unwrapped_.size() / 2; i++)
  {
    tangent_angle_unwrapped_[i] = tangent_angle_unwrapped_[i + tangent_angle_unwrapped_.size() / 2];
    tangent_angle_unwrapped_[i + tangent_angle_unwrapped_.size() / 2] =
        tangent_angle_unwrapped_[i] + direction * 2.0 * M_PI;
  }
}

double StaticTrackTrajectory::getArcLength(int i) const
{
  return (int(i / arc_length_.size()) * getMaxArcLength()) + arc_length_[i % arc_length_.size()];
}

void StaticTrackTrajectory::increaseTangentAngle()
{
  changeTangentAngle(1);
}

void StaticTrackTrajectory::decreaseTangentAngle()
{
  changeTangentAngle(-1);
}

double StaticTrackTrajectory::getMaxArcLength() const
{
  return arc_length_[arc_length_.size() - 1];
}

}  // namespace crs_controls
