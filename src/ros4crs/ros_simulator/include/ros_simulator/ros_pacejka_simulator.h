#ifndef ROS_SIMULATOR_ROS_PACEJKA_SIMULATOR_H
#define ROS_SIMULATOR_ROS_PACEJKA_SIMULATOR_H

#include <ros/ros.h>

#include "common/ros_simulator.h"
#include <memory>

#include "pacejka_model/pacejka_discrete.h"

#include <crs_msgs/car_input.h>
#include <crs_msgs/car_state_cart.h>
#include <ros_crs_utils/state_message_conversion.h>

#include <sensor_models/sensor_model.h>

#include <commons/static_track_trajectory.h>
#include <pacejka_model/pacejka_car_input.h>

#include "ros_simulator/delayed_publisher.h"

namespace ros_simulator
{

class PacejkaSimulator : public Simulator
{
private:
  std::unique_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model_;
  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publisher
  ros::Publisher gt_state_pub;
  // Subscriptions
  ros::Subscriber control_input_sub_;

  // Model state
  crs_models::pacejka_model::pacejka_car_input last_input_;
  crs_models::pacejka_model::pacejka_car_state current_state_;
  std::shared_ptr<NoiseModel> noise_model_;

  // Measurements
  std::vector<std::shared_ptr<crs_sensor_models::SensorModel<crs_models::pacejka_model::pacejka_car_state,
                                                             crs_models::pacejka_model::pacejka_car_input, 6>>>
      sensor_models_;  // list of sensor models e.g. vicon, imu, ...
  std::vector<DelayedPublisher> sensor_models_pub_;

  bool got_init_input = false;

  bool simulate_track_collision_ = true;
  // Only used for collision
  std::shared_ptr<crs_controls::StaticTrackTrajectory> static_track_trajectory_;
  bool collision_detected_ = false;

public:
  PacejkaSimulator(ros::NodeHandle nh, ros::NodeHandle nh_private);

  void advanceState(double timestep) override;
  void publishStates() override;
  void publishMeasurement(const std::string& key) override;
  void registerNoiseModel(std::shared_ptr<NoiseModel> noise_model) override;
  void printConfig() override;

  void inputCallback(crs_msgs::car_inputConstPtr input);
  void
  registerSensorModel(std::shared_ptr<crs_sensor_models::SensorModel<crs_models::pacejka_model::pacejka_car_state,
                                                                     crs_models::pacejka_model::pacejka_car_input, 6>>
                          sensor_model,
                      double delay);
};
}  // namespace ros_simulator
#endif
