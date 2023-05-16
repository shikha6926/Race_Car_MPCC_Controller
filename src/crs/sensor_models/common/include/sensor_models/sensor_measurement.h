#ifndef SENSOR_MODELS_SENSOR_MEASUREMENT_H
#define SENSOR_MODELS_SENSOR_MEASUREMENT_H

#include <Eigen/Core>
#include <string>

namespace crs_sensor_models
{
struct measurement
{
  std::string sensor_key;  // where measurement comes from e.g. Vicon
  Eigen::VectorXd measurement_data;
  double timestamp;
};
}  // namespace crs_sensor_models
#endif
