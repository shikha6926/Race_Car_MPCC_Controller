#include <ros_crs_utils/parameter_io.h>

namespace parameter_io
{

std::shared_ptr<crs_controls::StaticTrackTrajectory> loadTrackDescriptionFromParams(const ros::NodeHandle& nh)
{
  std::cout << "LOADING TRACK FROM: " << nh.getNamespace() << std::endl;
  std::vector<double> x_coords_;
  std::vector<double> y_coords_;
  std::vector<double> x_rate_;
  std::vector<double> y_rate_;
  std::vector<double> arc_length_;
  std::vector<double> tangent_angle_;
  std::vector<double> curvature_;
  double width;
  double density;

  if (!nh.getParam("xCoords", x_coords_))
    ROS_WARN_STREAM("Could not load track! Did not find x_coords for namespace: " << nh.getNamespace() << ".");

  if (!nh.getParam("yCoords", y_coords_))
    ROS_WARN_STREAM("Could not load track! Did not find x_coords for namespace: " << nh.getNamespace() << ".");

  nh.getParam("xRate", x_rate_);
  nh.getParam("yRate", y_rate_);
  nh.getParam("trackWidth", width);
  nh.getParam("tangentAngle", tangent_angle_);
  nh.getParam("curvature", curvature_);
  if (!nh.getParam("arcLength", arc_length_))
  {
    ROS_WARN_STREAM("Could not load arcLength! Did not find arcLength for namespace: " << nh.getNamespace() << ".");
  }
  nh.getParam("density", density);

  // Resize vectors
  // Track coordinates are doubled, cut in half and store one set.
  std::vector<double> x_coords(x_coords_.begin(), x_coords_.begin() + x_coords_.size() / 2);
  std::vector<double> y_coords(y_coords_.begin(), y_coords_.begin() + y_coords_.size() / 2);
  std::vector<double> x_rate(x_rate_.begin(), x_rate_.begin() + x_rate_.size() / 2);
  std::vector<double> y_rate(y_rate_.begin(), y_rate_.begin() + y_rate_.size() / 2);
  std::vector<double> arc_length(arc_length_.begin(), arc_length_.begin() + arc_length_.size() / 2);
  std::vector<double> tangent_angle(tangent_angle_.begin(), tangent_angle_.begin() + tangent_angle_.size() / 2);
  std::vector<double> curvature(curvature_.begin(), curvature_.begin() + curvature_.size() / 2);

  return std::make_shared<crs_controls::StaticTrackTrajectory>(x_coords, y_coords, x_rate, y_rate, width, curvature,
                                                               tangent_angle, arc_length, density);
}

}  // namespace parameter_io