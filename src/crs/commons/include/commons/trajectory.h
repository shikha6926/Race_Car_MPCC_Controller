#ifndef TRAJECTORY
#define TRAJECTORY

#include <Eigen/Core>
#include <vector>

namespace crs_controls
{

class Trajectory
{
protected:
  /**
   * @brief X coordinate of the trajectory
   *
   */
  std::vector<double> x_coords_;
  /**
   * @brief Y coordinates of the trajectory
   *
   */
  std::vector<double> y_coords_;

  int lastTrackIdx_ = 0;

  // edges of Voronoi Distribution
  std::vector<double> voronoi_edges_x_;
  std::vector<double> voronoi_edges_y_;

public:
  /**
   * @brief Construct a new Trajectory
   *
   * @param x_coord x_coordinates of the trajectory
   * @param y_coord y_coordinates of the trajectory
   */
  Trajectory(std::vector<double> x_coord, std::vector<double> y_coord) : x_coords_(x_coord), y_coords_(y_coord){};

  Trajectory(){};

  /**
   * @brief Returns the trajectory point at the given index.
   * @note this function performs index wrapping, allowing to pass indices that are negative or bigger than the
   * trajectory length
   *
   * @param i index to request
   * @return Eigen::Vector2d the trajectory point at position i
   */
  Eigen::Vector2d operator[](int i) const;

  /**
   * @brief Get the Closest Track Point Idx object
   *
   * @param query_point
   * @param precision
   * @return int
   */
  int getClosestTrackPointIdx(const Eigen::Vector2d& query_point, int precision = 16);

  /**
   * @brief Get the trajectory point that is closest to the given query point.
   * @note This function uses a binary search internally (first probing for the smallest distance using a step size of
   * "precision").
   *
   * @param query_point position to query closest trajectory point
   * @param precision probing step size for binary search
   * @return Eigen::Vector2d
   */
  Eigen::Vector2d getClosestTrackPoint(const Eigen::Vector2d& query_point, int precision = 16);

  /**
   * @brief Returnes the last closest point of the trajectory that was previously requested.
   *
   * @return const Eigen::Vector2d
   */
  const Eigen::Vector2d getLastRequestedTrackPoint() const;

  /**
   * @brief Get the Last Requested Track Angle (only for visualization)
   *
   * @return const double
   */
  virtual const double getLastRequestedTrackAngle() const = 0;

  /**
   * @brief Get the Voronoi Edges (only for visualization). Leave empty if unused.
   *
   * @return std::vetcor<double>
   */
  std::vector<double> getVorEdgesX();
  std::vector<double> getVorEdgesY();
};

}  // namespace crs_controls

#endif /* TRAJECTORY */
