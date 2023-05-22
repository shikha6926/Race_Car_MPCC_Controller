#ifndef SRC_CRS_COMMONS_INCLUDE_COMMONS_TRAJECTORY_SPLINE
#define SRC_CRS_COMMONS_INCLUDE_COMMONS_TRAJECTORY_SPLINE

#include <Eigen/Core>
#include <vector>
#include "commons/trajectory.h"

namespace crs_controls
{
class SplineTrajectory : public Trajectory
{
private:
  /**
   * @brief x,y coefficients of cubic splines
   */
  std::vector<double> X_coef_0_;
  std::vector<double> X_coef_1_;
  std::vector<double> X_coef_2_;
  std::vector<double> X_coef_3_;

  std::vector<double> Y_coef_0_;
  std::vector<double> Y_coef_1_;
  std::vector<double> Y_coef_2_;
  std::vector<double> Y_coef_3_;
  /**
   * @brief the rate of change in x direction of all points on track centerline
   *
   */
  std::vector<double> x_rate_;
  /**
   * @brief the rate of change in y direction of all points on track centerline
   *
   */
  std::vector<double> y_rate_;
  /**
   * @brief (1/radius, zero if straight) at of the track segment from the start to this index
   *
   */
  //std::vector<double> curvature_;
  /**
   * @brief the distance on the centerline from the start to this index
   *
   */
  //std::vector<double> arc_length_;

  /**
   * @brief Tangent angle of the track.
   * This vector is twice the size of the track to handle the end of a lap
   */
  //std::vector<double> tangent_angle_;

  /**
   * @brief Unwrapped tangent angle of the track.
   * This vector is twice the size of the track to handle the end of a lap
   */
  //std::vector<double> tangent_angle_unwrapped_;

  double phi_;

  /**
   * @brief The width of the track
   */
  double width_;

  /**
   * @brief The density of the centerline points
   *
   */
  double density_ = 2.9367;

  /**
   * @brief Unwraps the track angle over the full trajectory
   *
   
  void unwrapYawAngle();
  */

  /**
   * @brief Updates the track angle by either adding 2*pi (direction = 1) or subtracting it (direction = -1)
   *
   * @param direction +1 / -1
   
  void changeTangentAngle(int direction);
  */

public:
  /**
   * @brief Construct a new Track ManagerTrajectory
   *
   * @param x_coord x_coordinates of the track
   * @param y_coord y_coordinates of the track
   * @param X_coef_0 x_coeff of cubic spline
   * @param X_coef_1 x_coeff of cubic spline
   * @param X_coef_2 x_coeff of cubic spline
   * @param X_coef_3 x_coeff of cubic spline
   * @param Y_coef_0 Y_coeff of cubic spline
   * @param Y_coef_1 Y_coeff of cubic spline
   * @param Y_coef_2 Y_coeff of cubic spline
   * @param Y_coef_3 Y_coeff of cubic spline
   * @param density density of the track centerline points
   */
  SplineTrajectory(std::vector<double> x_coord, std::vector<double> y_coord, 
                    std::vector<double> X_coef_0, std::vector<double> X_coef_1,
                    std::vector<double> X_coef_2, std::vector<double> X_coef_3,
                    std::vector<double> Y_coef_0, std::vector<double> Y_coef_1,
                    std::vector<double> Y_coef_2, std::vector<double> Y_coef_3 
                    ): Trajectory(x_coord, y_coord)
    , X_coef_0_(X_coef_0)
    , X_coef_1_(X_coef_1)
    , X_coef_2_(X_coef_2)
    , X_coef_3_(X_coef_3)
    , Y_coef_0_(Y_coef_0)
    , Y_coef_1_(Y_coef_1)
    , Y_coef_2_(Y_coef_2)
    , Y_coef_3_(Y_coef_3)
  {
   // tangent_angle_unwrapped_.resize(tangent_angle_.size() * 2, 0.0);
    // Unwrap yaw angle of the full track
    //unwrapYawAngle();
  };

  /**
   * @brief Construct a new Track Manager
   *
   * @param x_coord x_coordinates of the track
   * @param y_coord y_coordinates of the track
   * @param x_rate x_rate of the track
   * @param y_rate y_rate of the track
   * @param width width of the track
   */
  SplineTrajectory(std::vector<double> x_coord, std::vector<double> y_coord, std::vector<double> x_rate,  // NOLINT
                        std::vector<double> y_rate, double width)                                              // NOLINT
    : Trajectory(x_coord, y_coord), x_rate_(x_rate), y_rate_(y_rate), width_(width){};

  /**
   * @brief Construct a new Track Manager
   *
   * @param x_coord x_coordinates of the track
   * @param y_coord y_coordinates of the track
   */
  SplineTrajectory(std::vector<double> x_coord, std::vector<double> y_coord) : Trajectory(x_coord, y_coord){};

  /**
   * @brief Construct a new Track Manager object
   *
   * @param points
   */
  //SplineTrajectory(const std::vector<Eigen::Vector2d>& points);

   /**
   * @brief Calculates the reference X,Y coordinates of cubic spline based on theta and coefficient index
   *
   * @return 2d vector of x, y coordinates
   */
  Eigen::Vector2d getRefCoords(double distance_on_track);
  
  /**
   * @brief Calculates the gradient at X,Y coordinates of cubic spline based on theta and coefficient index
   *
   * @return 2d vector of x, y coordinates
   */
  Eigen::Vector2d getGradient(double distance_on_track);

  /**
   * @brief Calculates the tangent angle at X,Y coordinates of cubic spline based on gradients
   *
   * @return tangent angle at given point
   */
  double getTangentAngle(double y_rate , double x_rate);

  /**
   * @brief Calculates the euclidean distance between two reference points
   *
   * @return euclidean distance
   */
  inline double euclidean2Dist(double x1, double y1, double x2, double y2);

  /**
   * @brief Get the index of closest spline based on reference point
   *
   * @return index of spline
   */
  int getclosestsplineindex(Eigen::Vector2d state_pt, SplineTrajectory spline_track);

  /**
   * @brief Get the distance on the track based on closest splne index and reference point
   *
   * @return distance on the track 
   */
  double gettheta(int spline_index, Eigen::Vector2d state_pt, SplineTrajectory spline_track);

  /**
   * @brief Get the Width of the track
   *
   * @return double
   
  double getWidth() const
  {
    return width_;
  }
  */
  
  const double getLastRequestedTrackAngle() const override;
  /**
   * @brief Get the Density of the centerline points
   *
   * @return double
   */

  double getDensity() const
  {
    return density_;
  }

  /**
   * @brief Get the Rate of the track at index i.
   *
   * @param i
   * @return Eigen::Vector2d
  
  Eigen::Vector2d getRate(int i) const
  {
    return Eigen::Vector2d(x_rate_[i % x_rate_.size()], y_rate_[i % y_rate_.size()]);
  }
  */

  /**
   * @brief Returns the current error of the query point to the track
   *
   * @param query_point point to query
   * @param precision probing step size for binary search
   * @return track_error track error
  
  track_error getTrackError(const Eigen::Vector2d& query_point, int precision = 16);
   */

  /**
   * @brief Gets the Center Line of the track
   *
   * @return std::vector<Eigen::Vector2d>
  
  std::vector<Eigen::Vector2d> getCenterLine() const;
 */

  /**
   * @brief Returnes the last angle of the track point that was previously requested.
   *
   * @return const double
  
  const double getLastRequestedTrackAngle() const override;
 */

  /**
   * @brief Get the Curvature of the track at a given index
   *
   * @param track_idx
   * @return const double
   
  const double getCurvature(int track_idx) const;
  */

  /**
   * @brief Get the Track Angle at a given index.
   * @note The track angle is unwrapped
   *
   * @param track_idx
   * @return const double
   
  const double getTrackAngle(int track_idx) const;
  */

  /**
   * @brief Get the Mean Curvature along a path starting from start_idx ranging for a total number of distance points
   *
   * @param start_idx start position
   * @param distance number of points to calculate curvature
   * @return const double
   
  const double getMeanCurvatureAlongPath(int start_idx, int distance) const;
  */

  /**
   * @brief Call this to notify when a lap has been finished. Adds 2*pi to each angle of the track
   *
  
  void increaseTangentAngle();
   */

  /**
   * @brief Call this to notify when a lap has been finished. Subtracts 2*pi to each angle of the track
   *
   
  void decreaseTangentAngle();
  */

  /**
   * @brief returns the arc length at index i
   *
   * @param i
   * @return double
   
  double getArcLength(int i) const;
  */

  /**
   * @brief Returns the max arc length
   *
   
  double getMaxArcLength() const;
  */

  

};

}  // namespace crs_controls

#endif /* SRC_CRS_COMMONS_INCLUDE_COMMONS_STATIC_TRACK_TRAJECTORY */
