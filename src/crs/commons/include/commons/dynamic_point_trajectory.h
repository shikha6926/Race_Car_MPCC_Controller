#ifndef DYNAMIC_POINT_TRAJECTORY
#define DYNAMIC_POINT_TRAJECTORY

#include <Eigen/Core>
#include <vector>
#include <commons/trajectory.h>

namespace crs_controls
{

class DynamicPointTrajectory : public Trajectory
{
public:
  /**
   * @brief Construct a new Trajectory
   *
   * @param x_coord x_coordinates of the trajectory
   * @param y_coord y_coordinates of the trajectory
   */
  DynamicPointTrajectory(std::vector<double> x_coord, std::vector<double> y_coord) : Trajectory(x_coord, y_coord){};

  DynamicPointTrajectory() : Trajectory(){};

  /**
   * @brief Resets the current trajectory
   *
   * @param x_coord
   * @param y_coord
   */
  void resetTrajectory(std::vector<double> x_coord, std::vector<double> y_coord);

  /**
   * @brief Resets the current trajectory
   *
   * @param pts
   */
  void resetTrajectory(std::vector<Eigen::Vector2d> pts);

  /**
   * @brief Resets the current voronoi partition
   *
   * @param x_edge
   * @param y_edge
   */
  void resetVorEdges(std::vector<double> x_edge, std::vector<double> y_edge);


  /**
   * @brief Get the Last Requested Track Angle (only for visualization)
   *
   * @return const double
   */
  const double getLastRequestedTrackAngle() const override;
};

}  // namespace crs_controls

#endif /* TRAJECTORY */
