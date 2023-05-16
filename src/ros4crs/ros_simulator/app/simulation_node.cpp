#include <memory>
#include <ros/ros.h>

#include "ros_simulator/common/ros_simulator.h"
#include "ros_simulator/gaussian_noise_model.h"
#include <ros_crs_utils/parameter_io.h>

#include <ros_simulator/component_registry.h>

ros_simulator::Simulator* simulator = nullptr;

//  =====================================================================
// ========================== TIMER CALLBACKS  ==========================
//  =====================================================================
ros::Time current_time;

/**
 * @brief Function gets calles in a specific intervall, depending on how fast the measurements are published
 *
 * @param event Ros Timer Event - Ignored
 * @param sensor name of sensor e.g. vicon
 */
void publishMeasurementsCallback(const ros::TimerEvent& event, const std::string& sensor)
{
  simulator->publishMeasurement(sensor);
}

/**
 * @brief Function gets called in a specific intervall. Propogates state in simulator (adds noise if needed) and
 * publishes ground truth state
 *
 * @param event Ros Timer Event - Ignored
 */
void advanceSimulator(const ros::TimerEvent& event)
{
  ros::Time now = ros::Time::now();

  if (current_time.toSec() == 0)  // First time
  {
    current_time = now;
  }

  double elapsed = (now.toSec() - current_time.toSec());  // timestep for model propagation
  current_time = now;
  simulator->advanceState(elapsed);  // propogate model and add noise if needed
  simulator->publishStates();
}

//  =====================================================================
// ========================== SIMULATOR SETUP  ==========================
//  =====================================================================
/**
 * @brief Initialize the pointer "simulator". If this function does not return 0, the main loop will be terminated
 *
 * @note This function expects that the rosparameters are set a specific way. E.g. for LEMANS_CAR:
 *         LEMANS_CAR/simulator_ros/ <-- All simulator specific parameters are here (see file
 * pacejka_car_simulator.yaml) LEMANS_CAR/model/ <-- All model specific parameters are here (see file model_params.yaml
 * in pacejka_model)
 *
 *
 * @param nh nodehandel pointing to /<NAMESPACE>/*  e.g LEMANS_CAR/
 * @param nh_private nodehandle pointing to /<NAMESPACE>/simulator_ros/*  e.g LEMANS_CAR/simulator_ros/
 * @param sensors_to_load list of sensor names that should be loaded e.g. vicon, imu, ... (see
 * pacejka_car_simulator.yaml - sensors/sensor_names)
 * @return int Status Code to return. Returns 0 if everything was ok
 */
int setupSimulator(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::vector<std::string>& sensors_to_load)
{
  std::string state_type;
  if (!nh_private.getParam("state_type", state_type))  // Load state type from params (pacejka_car_simulator.yaml)
  {
    ROS_ERROR_STREAM("Could not load state type from parameters. Aborting Simulator!");
    return 1;
  }

  std::string input_type;
  if (!nh_private.getParam("input_type", input_type))  // Load input type from params (pacejka_car_simulator.yaml)
  {
    ROS_ERROR_STREAM("Could not load input type from parameters. Aborting Simulator!");
    return 1;
  }

  if (sensors_to_load.empty())
    ROS_WARN_STREAM("Provided sensor list was empty!. Running without any sensors!");

  simulator = ros_simulator::resolveSimulator(nh, nh_private, input_type, state_type, sensors_to_load);

  if (!simulator)
  {
    ROS_ERROR_STREAM("Did not find simulator for state and input " << state_type << " , " << input_type);
    return 1;
  }
  return 0;
}

//  ========================== NOISE MODELs  ==========================
/**
 * @brief Creates a noise model based on the parameters specified in the nodehandle
 * @note requires /type parameter to be set.
 *
 * @param nh node handle pointing to the noise parameter (nh.getParam("type") must be valid)
 * @return std::shared_ptr<ros_simulator::NoiseModel> the noise model
 */
std::shared_ptr<ros_simulator::NoiseModel> getNoiseModelFromParameters(ros::NodeHandle nh)
{
  // Create noise model
  std::shared_ptr<ros_simulator::NoiseModel> noise_model;
  std::string noise_type;
  if (nh.getParam("type", noise_type))  // Load noise from rosparams (pacejka_car_simulator.yaml)
  {
    if (noise_type == "multivariate_gaussian")
    {
      int seed = 42;
      nh.getParam("seed", seed);  // Load seed for Gaussian noise from params (pacejka_car_simulator.yaml)

      Eigen::VectorXd mean;
      // Load mean for Gaussian noise from params (pacejka_car_simulator.yaml)
      if (parameter_io::getMatrixFromParams<-1, 1>(ros::NodeHandle(nh, "mean"), mean))  // NOLINT
      {
        noise_model.reset((ros_simulator::NoiseModel*)new ros_simulator::GaussianNoiseModel(seed, mean));
      }
      else
      {
        noise_model.reset((ros_simulator::NoiseModel*)new ros_simulator::GaussianNoiseModel(seed));
      }
    }
    else
    {
      ROS_WARN_STREAM("Invalid noise type:  " << noise_type << " No process noise added to simulator!");
    }
  }
  return noise_model;
}

/**
 * @brief Loads process noise model (e.g. multivariate_gaussian) from rosparameters and registers it with simulator.
 *
 * @param nh nodehandle pointing to /<NAMESPACE>/simulator_ros/*  e.g LEMANS_CAR/simulator_ros/
 */
void loadProcessNoise(ros::NodeHandle nh)
{
  // Load process noise model
  std::shared_ptr<ros_simulator::NoiseModel> noise_model =
      getNoiseModelFromParameters(ros::NodeHandle(nh, "process_noise"));
  if (noise_model)
    simulator->registerNoiseModel(noise_model);
}

/**
 * @brief Loads measurement noise model (e.g. multivariate_gaussian) from rosparameters and registers it with simulator.
 *
 * @param nh nodehandle pointing to /<NAMESPACE>/simulator_ros/*  e.g LEMANS_CAR/simulator_ros/
 * @param sensor_names name of the sensor to add noise
 */
void loadMeasuremntNoise(ros::NodeHandle nh, const std::vector<std::string>& sensor_names)
{
  for (auto sensor_name : sensor_names)  // Parse sensor models
  {
    // Load measurement noise model
    std::shared_ptr<ros_simulator::NoiseModel> noise_model =
        getNoiseModelFromParameters(ros::NodeHandle(nh, "sensors/" + sensor_name + "/measurement_noise"));
    if (noise_model)
      simulator->addMeasurementNoise(sensor_name, noise_model);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulation_node");
  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/simulator_ros/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/*

  std::vector<std::string> sensors_to_load;
  if (!nh_private.getParam("sensors/sensor_names", sensors_to_load))  // Load sensor from params
                                                                      // (pacejka_car_simulator.yaml)
    ROS_WARN_STREAM("Could not load sensors for simulator. Running without any sensors!");

  if (setupSimulator(nh, nh_private, sensors_to_load) == 1)
    return 1;

  loadProcessNoise(nh_private);

  std::vector<ros::Timer> timers;
  for (auto sensor_name : sensors_to_load)
  {
    double frequency;
    if (!nh_private.getParam("sensors/" + sensor_name + "/frequency",
                             frequency))  // Load frequency of sensor from params (pacejka_car_simulator.yaml)
    {
      ROS_WARN_STREAM("No sampling frequency defined for sensor "
                      << sensor_name
                      << ". This sensor will not publish anything!\n Make sure to set the frequency parameter");
      continue;
    }

    // Publish measurements at given frequency of respective sensor
    timers.push_back(
        nh.createTimer(ros::Duration(1 / frequency), boost::bind(publishMeasurementsCallback, _1, sensor_name)));
  }

  float simulator_frequency = 50.0;
  if (!nh_private.getParam("frequency", simulator_frequency))  // Load simulation frequency from params
                                                               // (pacejka_car_simulator.yaml)
    ROS_INFO_STREAM("No frequency for simulator set. Defaulting to 50Hz!");

  simulator->printConfig();
  // Start Simulator
  ros::Time current_time = ros::Time::now();

  // Advance simulation at simulation frequency
  ros::Timer simulator_timer = nh.createTimer(ros::Duration(1 / simulator_frequency), advanceSimulator);
  // Create async spinners
  ros::AsyncSpinner spinner(1 + sensors_to_load.size());
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
