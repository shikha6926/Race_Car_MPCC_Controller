#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <random>

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_continuous.h>
#include <pacejka_model/pacejka_discrete.h>
#include <pacejka_model/pacejka_params.h>
#include <dynamic_models/utils/data_conversion.h>
#include <sensor_models/sensor_measurement.h>

#include "lowpass_estimator/pacejka_lowpass_estimator.h"
#include <pacejka_model/tests/test_utils.h>

#define N_RUNS 10

#define ESTIMATION_STEPS 2000

#define X_PRECISION 0.001
#define Y_PRECISION 0.001
#define YAW_PRECISION 0.2
#define VX_PRECISION 0.5
#define VY_PRECISION 0.5
#define YAW_REATE_PRECISION 2

#define ESTIMATOR_FREQUENCY 200  // Hz
#define TRANSIENT_IGNORE 100

crs_sensor_models::measurement measureState(crs_models::pacejka_model::pacejka_car_state& state, double time)
{
  Eigen::MatrixXd measurement;
  measurement.setZero(3, 1);
  measurement(0) = state.pos_x;
  measurement(1) = state.pos_y;
  measurement(2) = state.yaw;
  crs_sensor_models::measurement m = { "vicon", measurement, time };
  return m;
}

/**
 * @brief Check if the kalman control callback works when there is no process noise.
 */
TEST(LowpassTest, testStateEstimateDrivingForward)
{
  for (int run = 0; run < N_RUNS; run++)
  {
    crs_models::pacejka_model::pacejka_car_state squared_error;

    crs_models::pacejka_model::pacejka_params params;
    loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");
    std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model =
        std::make_shared<crs_models::pacejka_model::DiscretePacejkaModel>(params);

    crs_models::pacejka_model::pacejka_car_state gt_dynamic_state = { 0, 0, 1, 0.1, 0, 0 };
    crs_models::pacejka_model::pacejka_car_input current_input = { 0.2, 0.05 };

    crs_estimators::lowpass_estimator::car_lowpass_parameters lp_params = {
      // Butterworth filter
      { 0.0029, 0.0087, 0.0087, 0.0029 },   { 1.0000, -2.3741, 1.9294, -0.5321 }, { 0.0029, 0.0087, 0.0087, 0.0029 },
      { 1.0000, -2.3741, 1.9294, -0.5321 }, { 0.0029, 0.0087, 0.0087, 0.0029 },   { 1.0000, -2.3741, 1.9294, -0.5321 },
      { 0.0029, 0.0087, 0.0087, 0.0029 },   { 1.0000, -2.3741, 1.9294, -0.5321 }
    };

    crs_estimators::lowpass_estimator::PacejkaLowpassEstimator estimator(lp_params, gt_dynamic_state);
    double time = 0;
    double timestep = 1.0 / ESTIMATOR_FREQUENCY;

    for (int i = 0; i < ESTIMATION_STEPS;
         i++)  // The ekf is always  one step behind, This is why this function is so weirdly constructed
    {
      estimator.measurementCallback(measureState(gt_dynamic_state, time));

      if (i > TRANSIENT_IGNORE)
      {
        auto estimate = estimator.getStateEstimate();
        squared_error.pos_x += std::pow(gt_dynamic_state.pos_x - estimate.pos_x, 2);
        squared_error.pos_y += std::pow(gt_dynamic_state.pos_y - estimate.pos_y, 2);
        squared_error.yaw += std::pow(gt_dynamic_state.yaw - estimate.yaw, 2);
        squared_error.vel_x += std::pow(gt_dynamic_state.vel_x - estimate.vel_x, 2);
        squared_error.vel_y += std::pow(gt_dynamic_state.vel_y - estimate.vel_y, 2);
        squared_error.yaw_rate += std::pow(gt_dynamic_state.yaw_rate - estimate.yaw_rate, 2);
      }

      // Input sampling
      current_input.torque += getRandomFloat(-0.5, 0.5);
      current_input.steer += getRandomFloat(-0.1, 0.1);
      if (current_input.steer > 0.4)
        current_input.steer = 0.4;
      if (current_input.steer < -0.4)
        current_input.steer = -0.4;
      if (current_input.torque < 0.1)
        current_input.torque = 0.1;
      if (current_input.torque > 1)
        current_input.torque = 1;

      gt_dynamic_state = model->applyModel(gt_dynamic_state, current_input, timestep);
      time += timestep;
    }

    auto mse = 1.0 / (ESTIMATION_STEPS - TRANSIENT_IGNORE) * squared_error;
    EXPECT_LT(mse.pos_x, std::pow(X_PRECISION, 2));
    EXPECT_LT(mse.pos_y, std::pow(Y_PRECISION, 2));
    EXPECT_LT(mse.yaw, std::pow(YAW_PRECISION, 2));
    EXPECT_LT(mse.vel_x, std::pow(VX_PRECISION, 2));
    EXPECT_LT(mse.vel_y, std::pow(VY_PRECISION, 2));
    EXPECT_LT(mse.yaw_rate, std::pow(YAW_REATE_PRECISION, 2));
  }
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}