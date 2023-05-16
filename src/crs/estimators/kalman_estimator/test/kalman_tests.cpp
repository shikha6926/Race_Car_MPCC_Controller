#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <random>

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_continuous.h>
#include <pacejka_model/pacejka_discrete.h>
#include <pacejka_model/tests/test_utils.h>

#include <pacejka_sensor_model/vicon_sensor_model.h>
#include <sensor_models/sensor_measurement.h>

#include "kalman_estimator/discrete_ekf.h"
#include <dynamic_models/utils/data_conversion.h>

#define N_RUNS 10

/**
 * @brief Check if the kalman control callback works when there is no process noise.
 */
TEST(EKFTest, testControlCallback)
{
  auto start = std::chrono::high_resolution_clock::now();
  int kalman_steps = 100;
  for (int run = 0; run < N_RUNS; run++)
  {
    crs_models::pacejka_model::pacejka_params params;
    loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");
    std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model =
        std::make_shared<crs_models::pacejka_model::DiscretePacejkaModel>(params);
    crs_models::pacejka_model::pacejka_car_state gt_dynamic_state;
    loadRandomState(gt_dynamic_state);
    crs_models::pacejka_model::pacejka_car_input current_input;

    crs_estimators::kalman::DiscreteEKF<crs_models::pacejka_model::pacejka_car_state,
                                        crs_models::pacejka_model::pacejka_car_input, 6, 2>
        ekf(model, gt_dynamic_state, Eigen::Matrix<double, 6, 6>::Identity());

    crs_models::pacejka_model::pacejka_car_state output_state = gt_dynamic_state;

    double timestep = 0.05;  // 20Hz
    double time = 0;
    loadRandomInput(current_input);
    ekf.controlInputCallback(current_input, time);
    for (int i = 0; i < kalman_steps;
         i++)  // The ekf is always one step behind, This is why this function is so weirdly constructed
    {
      loadRandomInput(current_input);
      ekf.controlInputCallback(current_input, time);

      if (i != 0)
      {
        // Check state equals
        Eigen::Matrix<double, 6, 1> gt = commons::convertToEigen(gt_dynamic_state);
        Eigen::Matrix<double, 6, 1> predicted = commons::convertToEigen(ekf.getStateEstimate());
        EXPECT_TRUE(gt.isApprox(predicted, 0.0001));
        if (!gt.isApprox(predicted))
        {
          std::cout << "[groundtruth]" << gt << ":\n[ekf] " << predicted << std::endl;
        }
      }

      gt_dynamic_state = model->applyModel(gt_dynamic_state, current_input, timestep);

      time += timestep;
    }
  }
  auto time_after = std::chrono::high_resolution_clock::now();
  std::cout << "Took "
            << std::chrono::duration_cast<std::chrono::milliseconds>(time_after - start).count() /
                   (kalman_steps * N_RUNS)
            << "ms/iter" << std::endl;
}

/**
 * @brief Check if the kalman measurements callback works when there is no measurement or process noise.
 *
 */
TEST(EKFTest, testMeasurementCallback)
{
  typedef crs_sensor_models::pacejka_sensor_models::ViconSensorModel ViconSensorModel;
  auto start = std::chrono::high_resolution_clock::now();

  int kalman_steps = 10;
  for (int run = 0; run < N_RUNS; run++)
  {
    crs_models::pacejka_model::pacejka_params params;
    loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");
    std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model =
        std::make_shared<crs_models::pacejka_model::DiscretePacejkaModel>(params);

    std::shared_ptr<ViconSensorModel> vicon_sensor_model = std::make_shared<ViconSensorModel>();
    vicon_sensor_model->setR(Eigen::Matrix3d::Identity());
    // States used for simulation
    crs_models::pacejka_model::pacejka_car_state gt_state;
    loadRandomState(gt_state);

    crs_estimators::kalman::DiscreteEKF<crs_models::pacejka_model::pacejka_car_state,
                                        crs_models::pacejka_model::pacejka_car_input, 6, 2>
        ekf(model, gt_state, Eigen::Matrix<double, 6, 6>::Identity());

    ekf.addSensorModel(ViconSensorModel::SENSOR_KEY, vicon_sensor_model);

    crs_models::pacejka_model::pacejka_car_input current_input;
    loadRandomInput(current_input);

    double timestep = 0.05;  // 20Hz
    double time = 0;

    ekf.controlInputCallback(current_input, time);  // This is here to initialize the time and internal state of the ekf
                                                    // to zero, ignore me

    for (int i = 0; i < kalman_steps;
         i++)  // The ekf is always one step behind, This is why this function is so weirdly constructed
    {
      loadRandomInput(current_input);
      ekf.controlInputCallback(current_input, time);
      // Model Simulation
      gt_state = model->applyModel(gt_state, current_input, timestep);
      time += timestep;

      loadRandomInput(current_input);
      ekf.controlInputCallback(current_input, time);

      gt_state = model->applyModel(gt_state, current_input, timestep);
      time += timestep;

      // Fake vicon measurement
      Eigen::Vector3d vicon_measurement;
      vicon_measurement[0] = gt_state.pos_x;
      vicon_measurement[1] = gt_state.pos_y;
      vicon_measurement[2] = gt_state.yaw;
      crs_sensor_models::measurement measurement = { ViconSensorModel::SENSOR_KEY, vicon_measurement, time };

      ekf.measurementCallback(measurement);

      // Check state equals
      Eigen::Matrix<double, 6, 1> s1 = commons::convertToEigen(gt_state);
      Eigen::Matrix<double, 6, 1> s2 = commons::convertToEigen(ekf.getStateEstimate());
      EXPECT_TRUE(s1.isApprox(s2));
      if (!s1.isApprox(s2))
      {
        std::cout << "[current]" << s1 << ":\n[ekf] " << s2 << std::endl;
      }
    }
  }
  auto time_after = std::chrono::high_resolution_clock::now();
  std::cout << "Took "
            << std::chrono::duration_cast<std::chrono::milliseconds>(time_after - start).count() /
                   (kalman_steps * N_RUNS)
            << "ms/iter" << std::endl;
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}