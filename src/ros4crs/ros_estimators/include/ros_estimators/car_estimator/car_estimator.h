#ifndef SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_PACEJKA_ESTIMATOR_PACEJKA_ESTIMATOR
#define SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_PACEJKA_ESTIMATOR_PACEJKA_ESTIMATOR

#include "ros_estimators/state_estimator_ros.h"

#include <ros/ros.h>
#include <crs_msgs/car_state_cart.h>
#include <crs_msgs/car_input.h>
#include <crs_msgs/lighthouse_sweep.h>

#include "ros_estimators/data_converter.h"
#include <estimators/base_estimator.h>
#include <estimators/model_based_estimator.h>
#include <ros_crs_utils/state_message_conversion.h>

namespace ros_estimators
{
/** Dummy struct for empty model */
struct empty_model
{
};
template <typename StateType, typename InputType, typename ModelType = empty_model>
class RosCarEstimator : public RosStateEstimator
{
public:
  // Construction.
  RosCarEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                  const std::vector<std::string> measurement_keys,
                  std::shared_ptr<crs_estimators::BaseEstimator<StateType>> base_estimator)
    : base_estimator(base_estimator), nh_(nh), nh_private_(nh_private)
  {
    // Check if base estimator is model based
    // Try to convert it to parent type
    auto model_based_est =
        std::dynamic_pointer_cast<crs_estimators::ModelBasedEstimator<StateType, InputType>>(base_estimator);
    if (model_based_est)
    {
      // old was safely casted to NewType
      model_based_estimator = model_based_est;
    }
    else
    {
      model_based_estimator = nullptr;
    }

    // Load config
    nh_private.getParam("max_callback_rate", max_measurement_rate_);
    nh_private.getParam("measurement_timeout_threshold", measurement_timeout_threshold_);
    if (!nh_private.getParam("world_frame", vicon_converter.world_frame))
      ROS_WARN_STREAM("Did not load parameter for world frame. Defaulting to: " << vicon_converter.world_frame);
    if (!nh_private.getParam("track_frame", vicon_converter.track_frame))
      ROS_WARN_STREAM("Did not load parameter for track frame. Defaulting to: " << vicon_converter.track_frame);
    if (!nh_private.getParam("update_track_transform", vicon_converter.update_track_transform))
      ROS_WARN_STREAM("Did not load parameter for update_track_transform. Defaulting to: "
                      << vicon_converter.update_track_transform);

    // Setup ros publishers
    state_estimate_pub_ = nh_private_.advertise<crs_msgs::car_state_cart>("best_state", 10);

    // Register callbacks for keys
    for (const auto& key : measurement_keys)
    {
      if (key == "vicon")
      {
        measurement_subs_.push_back(nh_.subscribe(key, 1, &RosCarEstimator::viconMeasurementCallback, this));
      }
      else if (key == "imu")
      {
        measurement_subs_.push_back(nh_.subscribe(key, 1, &RosCarEstimator::imuMeasurementCallback, this));
      }
      else if (key == "lighthouse")
      {
        measurement_subs_.push_back(nh_.subscribe(key, 10, &RosCarEstimator::lighthouseMeasurementCallback, this));
      }
      else
      {
        ROS_WARN_STREAM("Masurement key "
                        << key << " has no known ROS conversion registered in estimator. Key will be dropped!");
        continue;
      }
      sensor_last_timestamp_[key] = 0;  // Initialize sensor timestamp to zero.
    }

    assert(!measurement_subs_.empty() && "No Measurement topic provided for Estimator. Aborting!");

    // Set state to running. TODO: Toggle this externally using ros services. Probably different PR.
    is_running_ = true;
  };

  RosCarEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                  const std::vector<std::string> measurement_keys,
                  std::shared_ptr<crs_estimators::ModelBasedEstimator<StateType, InputType>> model_based_estimator)
    : RosCarEstimator(nh, nh_private, measurement_keys,
                      std::static_pointer_cast<crs_estimators::BaseEstimator<StateType>>(model_based_estimator))
  {
    // Regiser control input subscription if estimator is model based
    control_input_sub_ = nh_.subscribe("control_input", 1, &RosCarEstimator::controlInputCallback, this);
  };

  void checkMissingMeasurements(const long current_time)
  {
    if (startup_time == 0)
      startup_time = current_time;

    // Check for all measurement keys if data is received.
    for (const auto& sensor_entry : sensor_last_timestamp_)
    {
      const std::string sensor_name = sensor_entry.first;
      const float last_sensor_ts = sensor_entry.second;

      if (last_sensor_ts == 0)
      {
        sensor_last_timestamp_[sensor_name] = startup_time;
        continue;
      }

      if (current_time - last_sensor_ts > measurement_timeout_threshold_)
      {  // never received a measurement
         // or the latest is too old

        if (sensor_entry.second == startup_time)
        {
          ROS_WARN_STREAM_THROTTLE(5, "No measurement received from: " << sensor_name);
        }
        else
        {
          ROS_WARN_STREAM_THROTTLE(5, "Missing measurements from: " << sensor_name << ". Last message received: "
                                                                    << (current_time - last_sensor_ts)
                                                                    << " seconds ago.");
        }
      }
    }
  }

  bool checkSensorFrequency(const std::string sensor_name, const double timestamp)
  {
    if (!is_running_)
    {
      return false;
    }

    if (timestamp - sensor_last_timestamp_[sensor_name] < 1 / max_measurement_rate_)
    {
      return false;
    }
    sensor_last_timestamp_[sensor_name] = timestamp;

    return true;
  }

  void controlInputCallback(const crs_msgs::car_input::ConstPtr input_msg)
  {
    if (!is_running_)
      return;

    // Check if some measurements are ignored. This is here since we can not guarante that the measuremnt callbacks are
    // executed.
    checkMissingMeasurements(input_msg->header.stamp.toSec());

    // We have a valid input now.
    has_valid_input_ = true;

    last_input_ = *input_msg;
    std::dynamic_pointer_cast<crs_estimators::ModelBasedEstimator<StateType, InputType>>(base_estimator)
        ->controlInputCallback(message_conversion::convertToCrsInput<crs_msgs::car_input, InputType>(*input_msg),
                               input_msg->header.stamp.toSec());
  };

  void viconMeasurementCallback(const geometry_msgs::TransformStamped::ConstPtr msg)
  {
    if (!checkSensorFrequency("vicon", msg->header.stamp.toSec()))
      return;
    try
    {
      base_estimator->measurementCallback(vicon_converter.parseData2D(msg));
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  };

  void imuMeasurementCallback(const sensor_msgs::Imu::ConstPtr msg)
  {
    if (!checkSensorFrequency("imu", msg->header.stamp.toSec()))
      return;

    base_estimator->measurementCallback(parseImuData2D(msg));
  };

  void lighthouseMeasurementCallback(const crs_msgs::lighthouse_sweep::ConstPtr msg)
  {
    sensor_last_timestamp_["lighthouse"] = msg->header.stamp.toSec();

    base_estimator->measurementCallback(parseLighthouseSweep(msg));
  };

  void publishState() override;

  // Ugly. TODO, use getter and setter or similar
  std::shared_ptr<ModelType> model;

private:
  double max_measurement_rate_ = 50;            // Hz
  double measurement_timeout_threshold_ = 2.0;  // s

  std::map<std::string, double> sensor_last_timestamp_;

  // Keep track of missing calls.
  double startup_time = 0;

  // Flag to set state of estimator
  bool is_running_ = false;
  bool has_valid_input_ = false;

  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // The following needs to be initialized by child class
  // Publisher
  ros::Publisher state_estimate_pub_;
  // Subscriptions
  ros::Subscriber control_input_sub_;
  std::vector<ros::Subscriber> measurement_subs_;

  ViconConverter vicon_converter;
  std::shared_ptr<crs_estimators::BaseEstimator<StateType>> base_estimator;

  // Pointer to the model based estimator.
  // If the current estimator does not support models (i.e. predict) this will be set to nullptr
  std::shared_ptr<crs_estimators::ModelBasedEstimator<StateType, InputType>> model_based_estimator;

  crs_msgs::car_input last_input_;
};
}  // namespace ros_estimators

#endif /* SRC_ROS_ROS_ESTIMATORS_INCLUDE_ROS_ESTIMATORS_PACEJKA_ESTIMATOR_PACEJKA_ESTIMATOR */
