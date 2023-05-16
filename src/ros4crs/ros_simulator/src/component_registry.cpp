#include "ros_simulator/component_registry.h"
#include <ros_crs_utils/parameter_io.h>

#ifdef pacejka_model_FOUND
#include "ros_simulator/ros_pacejka_simulator.h"
#include <pacejka_sensor_model/imu_sensor_model.h>
#include <pacejka_sensor_model/vicon_sensor_model.h>
#endif

#ifdef kinematic_model_FOUND
#include "ros_simulator/ros_kinematic_simulator.h"
#include <kinematic_sensor_model/imu_sensor_model.h>
#include <kinematic_sensor_model/vicon_sensor_model.h>
#endif

namespace ros_simulator
{
Simulator* resolveSimulator(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& state_type,
                            const std::string& input_type, const std::vector<std::string>& sensors_to_load)
{
  //  ========================== PACEJKA MODEL  ==========================
#ifdef pacejka_model_FOUND
  if (state_type == "pacejka_car" && input_type == "pacejka_car")
  {
    // Create pacejka simulator
    auto* pacejka_simulator = new ros_simulator::PacejkaSimulator(nh, nh_private);

    for (auto sensor_name : sensors_to_load)  // Parse sensor models
    {
      std::string sensor_key;
      if (!nh_private.getParam("sensors/" + sensor_name + "/key", sensor_key))  // Load sensor key from params
                                                                                // (pacejka_car_simulator.yaml)
        sensor_key = sensor_name;

      // Measurement Delay
      double delay = 0.0;
      nh_private.getParam("sensors/" + sensor_name + "/delay", delay);

      if (sensor_key == crs_sensor_models::pacejka_sensor_models::ViconSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)

        // Create vicon sensor model using R
        std::shared_ptr<crs_sensor_models::pacejka_sensor_models::ViconSensorModel> vicon_sensor_model =
            std::make_shared<crs_sensor_models::pacejka_sensor_models::ViconSensorModel>(R);

        // Sensor used for simulation
        pacejka_simulator->registerSensorModel(vicon_sensor_model, delay);
      }
      else if (sensor_key == crs_sensor_models::pacejka_sensor_models::ImuSensorModel::SENSOR_KEY)
      {
        // ===================== IMU MODEL ===============================================
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)

        // Sensor used for simulation.
        // IMU sensor model requires model dynamics!
        crs_models::pacejka_model::pacejka_params params;
        parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(
            ros::NodeHandle(nh, "model/model_params/"), params);

        auto cont_model = std::make_shared<crs_models::pacejka_model::ContinuousPacejkaModel>(params);
        pacejka_simulator->registerSensorModel(
            std::make_shared<crs_sensor_models::pacejka_sensor_models::ImuSensorModel>(cont_model, R), delay);
      }
      // e.g. add imu model
      else
      {
        ROS_WARN_STREAM("Unknown sensor model " << sensor_name << ". Sensor model will not be loaded!");
      }
    }
    return pacejka_simulator;
  }
#endif

#ifdef kinematic_model_FOUND
  if (state_type == "kinematic_car" && input_type == "kinematic_car")
  {
    // Create kinematic_simulator simulator
    auto* kinematic_simulator = new ros_simulator::KinematicSimulator(nh, nh_private);
    for (auto sensor_name : sensors_to_load)  // Parse sensor models
    {
      std::string sensor_key;
      if (!nh_private.getParam("sensors/" + sensor_name + "/key", sensor_key))  // Load sensor key from params
                                                                                // (pacejka_car_simulator.yaml)
        sensor_key = sensor_name;
      // Measurement Delay
      double delay = 0.0;
      nh_private.getParam("sensors/" + sensor_name + "/delay", delay);

      if (sensor_key == crs_sensor_models::kinematic_sensor_models::ViconSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)

        // Create vicon sensor model using R
        std::shared_ptr<crs_sensor_models::kinematic_sensor_models::ViconSensorModel> vicon_sensor_model =
            std::make_shared<crs_sensor_models::kinematic_sensor_models::ViconSensorModel>(R);

        // Sensor used for simulation
        kinematic_simulator->registerSensorModel(vicon_sensor_model, delay);
      }
      else if (sensor_key == crs_sensor_models::kinematic_sensor_models::ImuSensorModel::SENSOR_KEY)
      {
        // ===================== IMU MODEL ===============================================
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh_private, "sensors/" + sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)

        // Sensor used for simulation.
        // IMU sensor model requires model dynamics!
        crs_models::kinematic_model::kinematic_params params;
        parameter_io::getModelParams<crs_models::kinematic_model::kinematic_params>(
            ros::NodeHandle(nh, "model/model_params/"), params);

        auto cont_model = std::make_shared<crs_models::kinematic_model::ContinuousKinematicModel>(params);
        kinematic_simulator->registerSensorModel(
            std::make_shared<crs_sensor_models::kinematic_sensor_models::ImuSensorModel>(cont_model, R), delay);
      }

      // e.g. add imu model
      else
      {
        ROS_WARN_STREAM("Unknown sensor model " << sensor_name << ". Sensor model will not be loaded!");
      }
    }
    return kinematic_simulator;
  }
#endif
  return nullptr;
}
}  // namespace ros_simulator