
#include <math.h>

#include "ros_estimators/component_registry.h"
#include "ros_estimators/data_converter.h"

#include <estimators/base_estimator.h>
#include <estimators/model_based_estimator.h>

#include <ros_crs_utils/parameter_io.h>
#include <ros_crs_utils/state_message_conversion.h>
#include <ros_estimators/car_estimator/car_estimator.h>

#ifdef pacejka_model_FOUND
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_discrete.h>
#include <pacejka_sensor_model/imu_sensor_model.h>
#include <pacejka_sensor_model/vicon_sensor_model.h>
#include <pacejka_sensor_model/lighthouse_sensor_model.h>

typedef crs_models::pacejka_model::pacejka_car_state pacejka_car_state;
typedef crs_models::pacejka_model::pacejka_car_input pacejka_car_input;
#endif

#ifdef kalman_estimator_FOUND
#include <kalman_estimator/discrete_ekf.h>
#endif

#ifdef kinematic_model_FOUND
#include <kinematic_model/kinematic_discrete.h>
#include <kinematic_sensor_model/imu_sensor_model.h>
#include <kinematic_sensor_model/vicon_sensor_model.h>
typedef crs_models::kinematic_model::kinematic_car_input kinematic_car_input;
typedef crs_models::kinematic_model::kinematic_car_state kinematic_car_state;
#endif

#ifdef lowpass_estimator_FOUND
#include <lowpass_estimator/pacejka_lowpass_estimator.h>
#endif

namespace ros_estimators
{

#ifdef kalman_estimator_FOUND
#ifdef kinematic_model_FOUND
/**
 * @brief Loads and registers all sensor models for the ekf filter
 *
 * @param nh node handle pointing to </sensors>
 * @param ekf the ekf with which to register the sensor model
 * @param model_params the model parameters. Used e.g. for imu model
 * @return std::vector<std::string> all sensor keys tthat have been loaded
 */
std::vector<std::string> loadSensorModels(
    const ros::NodeHandle& nh,
    const std::shared_ptr<crs_estimators::kalman::DiscreteEKF<kinematic_car_state, kinematic_car_input, 4, 2>>& ekf,
    const crs_models::kinematic_model::kinematic_params& model_params)
{
  // Load R for EKF
  std::vector<std::string> sensors_to_load;
  nh.getParam("sensors/sensor_names", sensors_to_load);
  for (auto sensor_name : sensors_to_load)  // Parse sensor models
  {
    if (nh.hasParam("sensors/" + sensor_name))
    {
      std::string sensor_key = sensor_name;  // default behaviour, use sensor_name as key if not provided
      nh.getParam("sensors/" + sensor_name + "/key", sensor_key);
      if (sensor_key == crs_sensor_models::kinematic_sensor_models::ViconSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh, "sensors/" + sensor_name + "/R"), R);
        std::shared_ptr<crs_sensor_models::kinematic_sensor_models::ViconSensorModel> vicon_sensor_model =
            std::make_shared<crs_sensor_models::kinematic_sensor_models::ViconSensorModel>(R);
        ekf->addSensorModel(crs_sensor_models::kinematic_sensor_models::ViconSensorModel::SENSOR_KEY,
                            vicon_sensor_model);
      }
      else if (sensor_key == crs_sensor_models::kinematic_sensor_models::ImuSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        // IMU model needs continuous kinematic
        auto cont_model = std::make_shared<crs_models::kinematic_model::ContinuousKinematicModel>(model_params);
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh, "sensors/" + sensor_name + "/R"), R);
        std::shared_ptr<crs_sensor_models::kinematic_sensor_models::ImuSensorModel> vicon_sensor_model =
            std::make_shared<crs_sensor_models::kinematic_sensor_models::ImuSensorModel>(cont_model, R);
        ekf->addSensorModel(crs_sensor_models::kinematic_sensor_models::ImuSensorModel::SENSOR_KEY, vicon_sensor_model);
      }
      else
      {
        ROS_WARN_STREAM("Unknown sensor model " << sensor_name << ". Sensor will not be used for ekf");
      }
    }
    else
    {
      ROS_WARN_STREAM("Missing key for sensor: " << sensor_name << ". Sensor will not be used for ekf");
    }
  }
  return sensors_to_load;
}
#endif

#ifdef pacejka_model_FOUND
/**
 * @brief Loads and registers all sensor models for the ekf filter
 *
 * @param nh node handle pointing to </sensors>
 * @param ekf the ekf with which to register the sensor model
 * @param model_params the model parameters. Used e.g. for imu model
 * @return std::vector<std::string> all sensor keys tthat have been loaded
 */
std::vector<std::string> loadSensorModels(
    const ros::NodeHandle& nh,
    const std::shared_ptr<crs_estimators::kalman::DiscreteEKF<pacejka_car_state, pacejka_car_input, 6, 2>>& ekf,
    const crs_models::pacejka_model::pacejka_params& model_params)
{
  // Load R for EKF
  std::vector<std::string> sensors_to_load;
  nh.getParam("sensor_names", sensors_to_load);
  for (auto sensor_name : sensors_to_load)  // Parse sensor models
  {
    if (nh.hasParam(sensor_name))
    {
      std::string sensor_key = sensor_name;  // default behaviour, use sensor_name as key if not provided
      nh.getParam(sensor_name + "/key", sensor_key);

      if (sensor_key == crs_sensor_models::pacejka_sensor_models::ViconSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh, sensor_name + "/R"),
                                                R);  // Load R from params (pacejka_car_simulator.yaml)
        std::shared_ptr<crs_sensor_models::pacejka_sensor_models::ViconSensorModel> vicon_sensor_model =
            std::make_shared<crs_sensor_models::pacejka_sensor_models::ViconSensorModel>(R);
        ekf->addSensorModel(crs_sensor_models::pacejka_sensor_models::ViconSensorModel::SENSOR_KEY, vicon_sensor_model);
      }
      else if (sensor_key == crs_sensor_models::pacejka_sensor_models::ImuSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        // IMU model needs continuous pacejka
        auto cont_model = std::make_shared<crs_models::pacejka_model::ContinuousPacejkaModel>(model_params);
        parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh, sensor_name + "/R"), R);
        std::shared_ptr<crs_sensor_models::pacejka_sensor_models::ImuSensorModel> vicon_sensor_model =
            std::make_shared<crs_sensor_models::pacejka_sensor_models::ImuSensorModel>(cont_model, R);
        ekf->addSensorModel(crs_sensor_models::pacejka_sensor_models::ImuSensorModel::SENSOR_KEY, vicon_sensor_model);
      }
      else if (sensor_key == crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel::SENSOR_KEY)
      {
        Eigen::Matrix<double, 2, 4> sensor_pos = Eigen::Matrix<double, 2, 4>::Zero();
        parameter_io::getMatrixFromParams<2, 4>(ros::NodeHandle(nh, sensor_name + "/sensor_pos"), sensor_pos);

        std::vector<std::string> base_stations;
        nh.getParam(sensor_name + "/base_stations", base_stations);
        for (auto base_station : base_stations)  // Parse sensor models
        {
          if (nh.hasParam(sensor_name + "/" + base_station))
          {
            std::string bs_node = sensor_name + "/" + base_station;
            Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
            int bs_ID = 0;
            Eigen::Vector3d bs_pos {0, 0, 2};
            Eigen::Matrix3d bs_rot;
            bs_rot << 0, 1, 0, 0, 0, -1, -1, 0, 0;
            double dt1 = 0;
            double dt2 = 0;
            
            parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh, bs_node + "/R"), R);
            nh.getParam(bs_node + "/bs_ID", bs_ID);
            parameter_io::getMatrixFromParams<3, 1>(ros::NodeHandle(nh, bs_node + "/P_bs"), bs_pos);
            parameter_io::getMatrixFromParams<3, 3>(ros::NodeHandle(nh, bs_node + "/R_bs"), bs_rot);
            nh.getParam(bs_node + "/dt1", dt1);
            nh.getParam(bs_node + "/dt2", dt2);

            double lp_tilt_1 = - M_PI / 6 - dt1;
            double lp_tilt_2 = M_PI / 6 - dt2;

            std::shared_ptr<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel> lighthouse_sensor_model_1 =
                std::make_shared<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel>(R, bs_pos, bs_rot, lp_tilt_1, sensor_pos);
            std::shared_ptr<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel> lighthouse_sensor_model_2 =
                std::make_shared<crs_sensor_models::pacejka_sensor_models::LighthouseSensorModel>(R, bs_pos, bs_rot, lp_tilt_2, sensor_pos);
            ekf->addSensorModel("lighthouse_" + std::to_string(bs_ID) + "_1", lighthouse_sensor_model_1);
            ekf->addSensorModel("lighthouse_" + std::to_string(bs_ID) + "_2", lighthouse_sensor_model_2);
          }
          else
          {
            ROS_WARN_STREAM("Missing key for base_station: " << base_station << ". Base station will not be used for ekf");
          }
        }
      }
      else
      {
        ROS_WARN_STREAM("Unknown sensor model " << sensor_name << ". Sensor will not be used for ekf");
      }
    }
    else
    {
      ROS_WARN_STREAM("Missing key for sensor: " << sensor_name << ". Sensor will not be used for ekf");
    }
  }
  return sensors_to_load;
}
#endif
#endif

#ifdef pacejka_model_FOUND
template <>
RosCarEstimator<pacejka_car_state, pacejka_car_input, ros_estimators::empty_model>*
resolveCarEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& estimator_type)
{
  pacejka_car_state initial_state =
      parameter_io::getState<pacejka_car_state>(ros::NodeHandle(nh_private, "initial_state"));

  if (estimator_type == "lowpass")
  {
#ifdef lowpass_estimator_FOUND
    crs_estimators::lowpass_estimator::car_lowpass_parameters parameters =
        parameter_io::getConfig<crs_estimators::lowpass_estimator::car_lowpass_parameters>(nh_private);
    auto estimator =
        std::make_shared<crs_estimators::lowpass_estimator::PacejkaLowpassEstimator>(parameters, initial_state);

    std::vector<std::string> sensors_to_load;
    nh_private.getParam("sensors/sensor_names", sensors_to_load);
    // 2) Create wrapper using pointer
    auto model_based_estimator = std::dynamic_pointer_cast<crs_estimators::BaseEstimator<pacejka_car_state>>(estimator);
    // 3) Return wrapper
    return new RosCarEstimator<crs_models::pacejka_model::pacejka_car_state,
                               crs_models::pacejka_model::pacejka_car_input>(nh, nh_private, sensors_to_load,
                                                                             model_based_estimator);
#endif
  }
  else if (estimator_type == "discrete_ekf")
  {
#ifdef kalman_estimator_FOUND

    // 1) Create shard pointer to (crs) ekf
    //  This has two stages: First load generic model parameters which are shared with all components
    Eigen::Matrix<double, 6, 6> Q;
    parameter_io::getMatrixFromParams<6, 6>(ros::NodeHandle(nh, "model/Q"), Q);
    crs_models::pacejka_model::pacejka_params params;
    parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(ros::NodeHandle(nh, "model/model_params/"),
                                                                            params);
    // Then overwrite specific parameters from local config (private nodehandle)
    parameter_io::getModelParams<crs_models::pacejka_model::pacejka_params>(
        ros::NodeHandle(nh_private, "model/model_params/"), params, false);
    parameter_io::getMatrixFromParams<6, 6>(ros::NodeHandle(nh_private, "model/Q"), Q);

    // Create Pacejka model
    std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model =
        std::make_shared<crs_models::pacejka_model::DiscretePacejkaModel>(params, Q);

    pacejka_car_input initial_input =
        parameter_io::getInput<pacejka_car_input>(ros::NodeHandle(nh_private, "initial_input"));

    Eigen::Matrix<double, 6, 6> P_init;
    if (!parameter_io::getMatrixFromParams<6, 6>(ros::NodeHandle(nh_private, "P_init"), P_init))
    {
      ROS_WARN_STREAM("No initial P set. Using identity");
      P_init = Eigen::Matrix<double, 6, 6>::Identity();
    }
    auto ekf = std::make_shared<crs_estimators::kalman::DiscreteEKF<pacejka_car_state, pacejka_car_input, 6, 2>>(
        model, initial_state, initial_input, P_init);
    //
    std::vector<std::string> loaded_sensors = loadSensorModels(ros::NodeHandle(nh_private, "sensors"), ekf, params);
    // 2) Create wrapper using pointer.  Downcast ekf to modelbased estimator type
    auto model_based_estimator = std::dynamic_pointer_cast<crs_estimators::ModelBasedEstimator<
        crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input>>(ekf);
    // 3) Return wrapper
    return new RosCarEstimator<crs_models::pacejka_model::pacejka_car_state,
                               crs_models::pacejka_model::pacejka_car_input>(nh, nh_private, loaded_sensors,
                                                                             model_based_estimator);
#endif
  }

  return nullptr;
}
#endif

#ifdef kinematic_model_FOUND

template <>
RosCarEstimator<kinematic_car_state, kinematic_car_input, crs_models::kinematic_model::kinematic_params>*
resolveCarEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& estimator_type)
{
  pacejka_car_state initial_state =
      parameter_io::getState<pacejka_car_state>(ros::NodeHandle(nh_private, "initial_state"));

  if (estimator_type == "discrete_ekf")
  {
#ifdef kalman_estimator_FOUND
    // Load model parameters from rosparameters (e.g. model_params.yaml  in kinematic_model)
    // This has two stages: First load generic model parameters which are shared with all components
    crs_models::kinematic_model::kinematic_params params;

    Eigen::Matrix<double, 4, 4> Q;
    parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh, "model/Q"), Q);
    parameter_io::getModelParams<crs_models::kinematic_model::kinematic_params>(
        ros::NodeHandle(nh, "model/model_params/"), params);
    // Then overwrite specific parameters from local config (private nodehandle)
    parameter_io::getModelParams<crs_models::kinematic_model::kinematic_params>(
        ros::NodeHandle(nh_private, "model/model_params/"), params, false);
    parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh_private, "model/Q"), Q);

    // Create kinematic model
    std::shared_ptr<crs_models::kinematic_model::DiscreteKinematicModel> model =
        std::make_shared<crs_models::kinematic_model::DiscreteKinematicModel>(params, Q);

    kinematic_car_state initial_state =
        parameter_io::getState<kinematic_car_state>(ros::NodeHandle(nh_private, "initial_state"));

    kinematic_car_input initial_input =
        parameter_io::getInput<kinematic_car_input>(ros::NodeHandle(nh_private, "initial_input"));

    // Create EKF pointer
    Eigen::Matrix<double, 4, 4> P_init;
    if (!parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh_private, "P_init"), P_init))
    {
      ROS_WARN_STREAM("No initial P set. Using identity");
      P_init = Eigen::Matrix<double, 4, 4>::Identity();
    }
    auto ekf = std::make_shared<crs_estimators::kalman::DiscreteEKF<kinematic_car_state, kinematic_car_input, 4, 2>>(
        model, initial_state, initial_input, P_init);
    //
    std::vector<std::string> loaded_sensors = loadSensorModels(ros::NodeHandle(nh_private, "sensors"), ekf, params);
    // 2) Create wrapper using pointer.  Downcast ekf to modelbased estimator type
    auto model_based_estimator =
        std::dynamic_pointer_cast<crs_estimators::ModelBasedEstimator<kinematic_car_state, kinematic_car_input>>(ekf);
    // 3) Return wrapper
    auto estimator =
        new RosCarEstimator<kinematic_car_state, kinematic_car_input, crs_models::kinematic_model::kinematic_params>(
            nh, nh_private, loaded_sensors, model_based_estimator);

    crs_models::kinematic_model::kinematic_params* param_ptr = new crs_models::kinematic_model::kinematic_params();
    *param_ptr = params;
    estimator->model.reset(param_ptr);

    return estimator;
#endif
  }

  return nullptr;
}
#endif

RosStateEstimator* resolveEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& state_type,
                                    const std::string& input_type, const std::string& estimator_type)
{
  if (state_type == "pacejka_car")
  {
#ifdef pacejka_model_FOUND
    if (input_type == "pacejka_car" || estimator_type == "lowpass")
    {
      return resolveCarEstimator<pacejka_car_state, pacejka_car_input>(nh, nh_private, estimator_type);
    }
#endif
  }
  else if (state_type == "kinematic_car")
  {
#ifdef kinematic_model_FOUND
    if (input_type == "kinematic_car" || estimator_type == "lowpass")
      return resolveCarEstimator<kinematic_car_state, kinematic_car_input,
                                 crs_models::kinematic_model::kinematic_params>(nh, nh_private, estimator_type);
#endif
  }
  return nullptr;
}
}  // namespace ros_estimators
