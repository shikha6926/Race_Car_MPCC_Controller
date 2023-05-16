#include "ros_simulator/ros_kinematic_simulator.h"
#include <geometry_msgs/TransformStamped.h>
#include <kinematic_sensor_model/imu_sensor_model.h>
#include <kinematic_sensor_model/vicon_sensor_model.h>
#include <ros/console.h>
#include <ros_crs_utils/parameter_io.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

namespace ros_simulator
{

KinematicSimulator::KinematicSimulator(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  // Load model parameters from rosparameters (e.g. model_params.yaml  in kinematic_model)
  parameter_io::getModelParams<crs_models::kinematic_model::kinematic_params>(
      ros::NodeHandle(nh_, "model/model_params/"), params);

  Eigen::Matrix<double, 4, 4> Q = Eigen::Matrix<double, 4, 4>::Identity();
  if (!parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh_, "model/Q"), Q))  // Load Q from rosparameters (e.g.
                                                                                     // model_params.yaml  in
                                                                                     // kinematic_model)
  {
    ROS_WARN_STREAM("Could not load Q Matrix for provided Model. Using to Identity Matrix");
  }

  // Create a kinematic model
  model_ = std::make_unique<crs_models::kinematic_model::DiscreteKinematicModel>(params, Q);

  gt_state_pub = nh_private_.advertise<crs_msgs::car_state_cart>("gt_state", 10);
  control_input_sub_ = nh_private_.subscribe("control_input", 10, &KinematicSimulator::inputCallback, this);

  std::vector<double> initial_input;
  if (nh_private_.getParam("initial_input", initial_input))  // Load initial input from rosparameters (e.g.
                                                             // kinematic_car_simulator.yaml)
  {
    last_input_.steer = initial_input[0];
    last_input_.torque = initial_input[1];
  }
  else
  {
    ROS_WARN_STREAM("No initial input set. Using steer: 0, torque: 0.4");
    last_input_.steer = 0.0;
    last_input_.torque = 0.4;
  }

  std::vector<double> initial_state;
  if (nh_private_.getParam("initial_state", initial_state))  // Load initial state from rosparameters (e.g.
                                                             // kinematic_car_simulator.yaml)
  {
    current_state_.pos_x = initial_state[0];
    current_state_.pos_y = initial_state[1];
    current_state_.yaw = initial_state[2];
    current_state_.velocity = initial_state[3];
  }
  else
  {
    ROS_WARN_STREAM("No initial state set. Using [0, 0, 0, 0.5]");
    current_state_.pos_x = 0;
    current_state_.pos_y = 0;
    current_state_.yaw = 0;
    current_state_.velocity = 0.5;
  }

  nh_private_.getParam("do_collision_checks", simulate_track_collision_);
  if (simulate_track_collision_)
  {
    static_track_trajectory_ = parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track"));
  }
}

void KinematicSimulator::printConfig()
{
  ROS_INFO_STREAM("Started kinematic Car Simulator!\n"
                  << "Initial State:\n"
                  << current_state_ << "\nInitial Input " << last_input_ << "\n\nUsing Noise:\n"
                  << (noise_model_ ? "Yes" : "No") << "\n\nProcess Noise Cov Matrix:\n"
                  << model_->getQ());

  ROS_INFO_STREAM("Registered Sensor Models:\n");
  for (const auto sensor_model : sensor_models_)
  {
    ROS_INFO_STREAM("Key: " << sensor_model->getKey() << "\n R:\n" << sensor_model->getR());
  }
}

/**
 * @brief Add process noise model to the simulator
 *
 * @param noise_model
 */
void KinematicSimulator::registerNoiseModel(std::shared_ptr<NoiseModel> noise_model)
{
  noise_model_ = noise_model;
}

/**
 * @brief Propagates model in simulation (adds process noise if needed)
 *
 * @param timestep time for which model is propagated
 */
void KinematicSimulator::advanceState(double timestep)
{
  if (simulate_track_collision_)
  {
    bool collided = static_track_trajectory_->getTrackError(Eigen::Vector2d(current_state_.pos_x, current_state_.pos_y))
                        .lateral_error > static_track_trajectory_->getWidth() / 2;

    if (collided)
    {
      ROS_WARN_THROTTLE(1, "Track collision in simulator detected!");

      if (!collision_detected_)  // State swapped from no collision - collision)
      {
        if (current_state_.velocity > 0)    // Hit the track driving forward (normal situation)
          current_state_.velocity = -0.05;  // Can not be zero, otherwise we get nan issues
        else                                // Hit the track driving backward
          current_state_.velocity = 0.05;   // Can not be zero, otherwise we get nan issues
      }

      current_state_.velocity = 0;
    }
    collision_detected_ = collided;
  }

  if (!got_init_input)
    return;

  current_state_ = model_->applyModel(current_state_, last_input_, timestep);
  if (noise_model_)
  {
    // Sample noise. Note Q is defined in noise per time
    Eigen::MatrixXd noise = noise_model_->sampleNoiseFromCovMatrix(timestep * model_->getQ());
    current_state_.pos_x += noise(0);
    current_state_.pos_y += noise(1);
    current_state_.yaw += noise(2);
    current_state_.velocity += noise(3);
  }
}

/**
 * @brief sets inputs received from ros message
 *
 * @param input
 */
void KinematicSimulator::inputCallback(crs_msgs::car_inputConstPtr input)
{
  last_input_.steer = input->steer;
  last_input_.torque = input->torque;
  got_init_input = true;
}

/**
 * @brief publish ground truth state of simulation model
 *
 */
void KinematicSimulator::publishStates()
{
  gt_state_pub.publish(message_conversion::convertStateToRosMsg<crs_msgs::car_state_cart,
                                                                crs_models::kinematic_model::kinematic_car_state,
                                                                message_conversion::kinematic_information>(
      current_state_, { last_input_, params }));
}

/**
 * @brief Add sensor model to list of sensor models
 *
 * @param sensor_model
 */
void KinematicSimulator::registerSensorModel(
    std::shared_ptr<crs_sensor_models::SensorModel<crs_models::kinematic_model::kinematic_car_state,
                                                   crs_models::kinematic_model::kinematic_car_input, 4>>
        sensor_model,
    double delay)
{
  sensor_models_.push_back(sensor_model);  // append sensor_model to list of all sensor models
  std::string key = sensor_model->getKey();

  if (key == crs_sensor_models::kinematic_sensor_models::ViconSensorModel::SENSOR_KEY)
  {
    auto vicon_pub = nh_private_.advertise<geometry_msgs::TransformStamped>(key, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, vicon_pub, delay)));
  }
  else if (key == crs_sensor_models::kinematic_sensor_models::ImuSensorModel::SENSOR_KEY)
  {
    auto pub = nh_private_.advertise<sensor_msgs::Imu>(key, 10);
    sensor_models_pub_.push_back(std::move(DelayedPublisher(nh_private_, pub, delay)));
  }
  else
  {
    ROS_WARN_STREAM("Unknown sensor key " << key);
  }
}

/**
 * @brief publish sensor measurements
 *
 * @param key key of measurement that should be published
 */
void KinematicSimulator::publishMeasurement(const std::string& key)
{
  for (int sensor_idx = 0; sensor_idx < sensor_models_.size(); sensor_idx++)  // iterate over all publishers
  {
    auto sensor_model = sensor_models_[sensor_idx];
    if (sensor_model->getKey() != key)  // Check if we want to publish this sensor
      continue;

    auto publisher = &sensor_models_pub_[sensor_idx];
    // Parse measurement (Eigen Vector) to custom ros message to publish it
    Eigen::MatrixXd measurement = sensor_model->applyModel(current_state_, last_input_);

    Eigen::MatrixXd noise;
    // sensor_name_to_noise_model_ of the form: {'vicon': GaussianNoise, 'imu': GaussianNoise}
    auto sensor_iter = sensor_name_to_noise_model_.find(key);
    if (sensor_iter != sensor_name_to_noise_model_.end())                           // Key was found
      noise = sensor_iter->second->sampleNoiseFromCovMatrix(sensor_model->getR());  // get R from noise model
    else
      noise.setZero(sensor_model->getR().rows(), 1);  // no noise

    // ========== VICON ==========

    if (sensor_model->getKey() == crs_sensor_models::kinematic_sensor_models::ViconSensorModel::SENSOR_KEY)
    {
      geometry_msgs::TransformStamped msg;

      // Parse vicon measurements to ros messagex to publish it
      msg.transform.translation.x = measurement(0) + noise(0);  // x positiion
      msg.transform.translation.y = measurement(1) + noise(1);  // y position
      // yaw angle, ros msg wants quaternion not euler angles
      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, measurement(2) + noise(2));
      msg.transform.rotation.x = myQuaternion.getX();
      msg.transform.rotation.y = myQuaternion.getY();
      msg.transform.rotation.z = myQuaternion.getZ();
      msg.transform.rotation.w = myQuaternion.getW();

      msg.header.stamp = ros::Time::now();
      publisher->publish(msg);
    }
    // ========== IMU ==========

    else if (sensor_model->getKey() == crs_sensor_models::kinematic_sensor_models::ImuSensorModel::SENSOR_KEY)
    {
      sensor_msgs::Imu msg;
      msg.angular_velocity.z = measurement(0) + noise(0);
      msg.linear_acceleration.x = measurement(1) + noise(1);
      msg.linear_acceleration.y = measurement(2) + noise(2);
      msg.header.stamp = ros::Time::now();
      publisher->publish(msg);
    }
  }
}
};  // namespace ros_simulator