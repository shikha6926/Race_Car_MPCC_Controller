#ifndef ROS_SIMULATOR_COMMON_ROS_SIMULATOR_H
#define ROS_SIMULATOR_COMMON_ROS_SIMULATOR_H

#include "noise_model.h"
#include <memory>
#include <map>

namespace ros_simulator
{
class Simulator
{
public:
  virtual void advanceState(double timestep) = 0;
  virtual void publishStates() = 0;
  virtual void publishMeasurement(const std::string& key) = 0;
  virtual void registerNoiseModel(std::shared_ptr<NoiseModel> noise_model) = 0;
  virtual void printConfig() = 0;

  void addMeasurementNoise(std::string measurement_name, std::shared_ptr<NoiseModel> noise_model)
  {
    sensor_name_to_noise_model_.insert(
        std::pair<std::string, std::shared_ptr<NoiseModel>>(measurement_name, noise_model));
  }

protected:
  std::map<std::string, std::shared_ptr<NoiseModel>> sensor_name_to_noise_model_;
};
}  // namespace ros_simulator
#endif
