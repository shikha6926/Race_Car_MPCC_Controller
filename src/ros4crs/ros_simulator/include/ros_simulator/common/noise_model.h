#ifndef ROS_SIMULATOR_COMMON_NOISE_MODEL_H
#define ROS_SIMULATOR_COMMON_NOISE_MODEL_H

#include <Eigen/Core>

namespace ros_simulator
{
class NoiseModel
{
public:
  virtual Eigen::MatrixXd sampleNoiseFromCovMatrix(const Eigen::MatrixXd& cov_mat) = 0;
};
}  // namespace ros_simulator
#endif
