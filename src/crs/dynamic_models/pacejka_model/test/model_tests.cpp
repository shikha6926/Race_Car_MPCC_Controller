#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <math.h>
#include <random>

#include "pacejka_model/pacejka_car_input.h"
#include "pacejka_model/pacejka_car_state.h"
#include "pacejka_model/pacejka_continuous.h"
#include "pacejka_model/pacejka_discrete.h"
#include "pacejka_model/pacejka_params.h"

#include "pacejka_model/tests/test_utils.h"
#include <dynamic_models/utils/data_conversion.h>
#include <experimental/filesystem>

#define N_ITER 100

/**
 * @brief Checks if the apply model functions throws errors.
 * Currently, this check if pretty useless. TODO(sabodmer) Check with GT dynamics or sth.
 */
TEST(PacejkaTests, testApplyDynamics)
{
  crs_models::pacejka_model::pacejka_params params;
  loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");
  crs_models::pacejka_model::ContinuousPacejkaModel model(params);
  crs_models::pacejka_model::pacejka_car_state current_state;
  loadRandomState(current_state);
  crs_models::pacejka_model::pacejka_car_input current_input;
  loadRandomInput(current_input);
  crs_models::pacejka_model::pacejka_car_state output_rate = model.applyModel(current_state, current_input);
}

/**
 * @brief Checks if the discrete system drives forward
 *        Tests the casadi integrator by calling applyModel()
 */
TEST(PacejkaTests, testDiscreteDriveForward)
{
  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::pacejka_model::pacejka_params params;
    loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");
    crs_models::pacejka_model::DiscretePacejkaModel model(params);
    // only has a position and forward velocity
    crs_models::pacejka_model::pacejka_car_state current_state = {
      getRandomFloat(-5, 5), getRandomFloat(-5, 5), 0, getRandomFloat(0.4, 2), 0, 0
    };
    crs_models::pacejka_model::pacejka_car_input current_input = { getRandomFloat(0.3, 1),
                                                                   0 };  // only apply torque input

    for (int step = 0; step < 5; step++)
    {
      double ts = getRandomFloat(0.05, 0.1);
      auto next_state = model.applyModel(current_state, current_input, ts);
      EXPECT_LT(current_state.pos_x, next_state.pos_x);
      // Move current state
      current_state = next_state;
    }
  }
}

/**
 * @brief Caclucates the jacobian of the discrete system
 */
TEST(PacejkaTests, testDiscreteJacobian)
{
  crs_models::pacejka_model::pacejka_params params;
  loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");
  crs_models::pacejka_model::DiscretePacejkaModel model(params);
  crs_models::pacejka_model::pacejka_car_state current_state = {
    getRandomFloat(-5, 5), getRandomFloat(-5, 5), 0, getRandomFloat(0.4, 2), 0, 0
  };
  crs_models::pacejka_model::pacejka_car_input current_input = { getRandomFloat(0.3, 1), 0 };
  crs_models::pacejka_model::pacejka_car_state output_state = { 0, 0, 0, 0, 0, 0 };

  Eigen::Matrix<double, 6, 6> F;
  Eigen::Matrix<double, 6, 2> D;
  model.getJacobian(current_state, current_input, 0.1, F, D);
}

/**
 * @brief Caclucates the jacobian of the discrete system
 */
TEST(PacejkaTests, testIntegrator)
{
  auto start = std::chrono::high_resolution_clock::now();
  crs_models::pacejka_model::pacejka_params params;
  params.lr = 0;
  params.lf = 0;
  params.m = 1;
  params.I = 1;
  params.Df = 0;
  params.Cf = 0;
  params.Bf = 0;
  params.Dr = 0;
  params.Cr = 0;
  params.Br = 0;
  params.Cm1 = 0;
  params.Cm2 = 1;
  params.Cd = 0;
  params.Croll = 0;
  /* With these parameters, dynamics are reduced to

     f(x) = [
      v_x*cos(yaw) - v_y*sin(yaw),
      v_x * sin(yaw) + v_y * cos(yaw),
      yaw_rate,
      - v_x * torque - 1 * v_y * yaw_rate
      - 1 * v_x * yaw_rate,
      0
  ] */
  crs_models::pacejka_model::DiscretePacejkaModel model(params);
  for (int run = 0; run < N_ITER; run++)
  {
    crs_models::pacejka_model::pacejka_car_state current_state = { getRandomFloat(-5, 5),  0, 0,
                                                                   getRandomFloat(0.4, 2), 0, 0 };
    crs_models::pacejka_model::pacejka_car_input current_input = { getRandomFloat(0.3, 1), 0 };

    // Equation is now (Assuming yaw = 0 const)
    // x_dot_dot = -x_dot * torque
    // --> x_dot(t) = x_dot(0) * e^-(torque*t)
    double ts = getRandomFloat(0.001, 0.2);
    auto vx_gt_solution = current_state.vel_x * exp(-current_input.torque * ts);
    crs_models::pacejka_model::pacejka_car_state output_state = model.applyModel(current_state, current_input, ts);
    auto vx_integrator_solution = output_state.vel_x;
    double squared_error = (vx_gt_solution - vx_integrator_solution) * (vx_gt_solution - vx_integrator_solution);
    EXPECT_LT(squared_error, 0.0001);
  }
  auto time_after = std::chrono::high_resolution_clock::now();
  std::cout << "Took " << std::chrono::duration_cast<std::chrono::microseconds>(time_after - start).count() / N_ITER
            << "us/iter" << std::endl;
}

/**
 * @brief Checks that the jacobian (currently only df/dx and not df/du) is correctly calculated using casadi
 */
TEST(PacejkaTests, testContinousJacobian)
{
  auto start = std::chrono::high_resolution_clock::now();
  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::pacejka_model::pacejka_params params;
    loadRandomPacejkaParams(params, "params/example_pacejka_params.yaml");
    crs_models::pacejka_model::ContinuousPacejkaModel pacejka_model(params);
    crs_models::ContinuousDynamicModel<crs_models::pacejka_model::pacejka_car_state,
                                       crs_models::pacejka_model::pacejka_car_input, 6, 2>& model = pacejka_model;

    crs_models::pacejka_model::pacejka_car_state state;
    loadRandomState(state);
    crs_models::pacejka_model::pacejka_car_input input;
    loadRandomInput(input);

    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 2> B;
    model.getNumericalJacobian(state, input, A, B);

    // Matlab jacobian
    double sinY = std::sin(state.yaw);
    double cosY = std::cos(state.yaw);
    double ar = std::atan2(-state.vel_y + params.lr * state.yaw_rate, state.vel_x);
    double af = input.steer + std::atan2(-state.vel_y - params.lf * state.yaw_rate, state.vel_x);
    double br_ar = params.Br * ar;
    double tmpR1 = params.Br * params.Cr * params.Dr * std::cos(params.Cr * std::atan(br_ar)) /
                   (((state.vel_y - state.yaw_rate * params.lr) * (state.vel_y - state.yaw_rate * params.lr) +
                     state.vel_x * state.vel_x) *
                    (br_ar * br_ar + 1));
    double tmpR2 = tmpR1 * (state.vel_y - state.yaw_rate * params.lr);
    double tmpR3 = tmpR1 * state.vel_x;
    double tmpR4 = tmpR3 * params.lr;
    double bf_af = params.Bf * af;
    double tmpF0 = params.Bf * params.Cf * params.Df * std::cos(params.Cf * std::atan(bf_af)) /
                   (((state.vel_y + state.yaw_rate * params.lf) * (state.vel_y + state.yaw_rate * params.lf) +
                     state.vel_x * state.vel_x) *
                    (bf_af * bf_af + 1));
    double tmpF1 = std::cos(input.steer) * tmpF0;
    double tmpF2 = tmpF1 * (state.vel_y + state.yaw_rate * params.lf);
    double tmpF3 = tmpF1 * state.vel_x;
    double tmpF4 = tmpF3 * params.lf;
    double tmpF5 = tmpF0 * std::sin(input.steer);
    // Jacobian
    Eigen::Matrix<double, 6, 6> J;
    J << 0.0, 0.0, -1.0 * (sinY * state.vel_x + cosY * state.vel_y), 1.0 * cosY, -1.0 * sinY, 0.0, 0.0, 0.0,
        1.0 * (state.vel_x * cosY - state.vel_y * sinY), 1.0 * sinY, 1.0 * cosY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0,
        0.0 - (1.0 / params.m) * (2 * params.Cd * state.vel_x + params.Cm2 * input.torque +
                                  (state.vel_y + state.yaw_rate * params.lf) * tmpF5),
        (1.0 / params.m) * (params.m * state.yaw_rate + state.vel_x * tmpF5),
        (1.0 / params.m) * (params.m * state.vel_y + params.lf * state.vel_x * tmpF5), 0.0, 0.0, 0.0,
        (1.0 / params.m) * (-params.m * state.yaw_rate + tmpR2 + tmpF2), 0.0 - (1.0 / params.m) * (tmpR3 + tmpF3),
        -(1.0 / params.m) * (params.m * state.vel_x - tmpR4 + tmpF4), 0.0, 0.0, 0.0,
        -(1.0 / params.I) * (params.lr * tmpR2 - params.lf * tmpF2),
        (1.0 / params.I) * (params.lr * tmpR3 - params.lf * tmpF3),
        0.0 - (1.0 / params.I) * (params.lr * tmpR4 + params.lf * tmpF4);

    EXPECT_TRUE(J.isApprox(A));

    if (!J.isApprox(A))
    {
      std::cout << params << std::endl;
      std::cout << state << std::endl;
      std::cout << input << std::endl;
      for (int i = 0; i < J.rows(); i++)
      {
        for (int j = 0; j < J.cols(); j++)
        {
          if (std::abs(J(i, j) - A(i, j)) > 0.0001)
          {
            std::cout << "missmatch at (" << i << "," << j << "): " << A(i, j) << " != " << J(i, j) << std::endl;
          }
        }
      }
      std::cout << "J:" << J << std::endl;
      std::cout << "===" << std::endl;
      std::cout << "A:" << A << std::endl;
      break;
    }
  }
  auto time_after = std::chrono::high_resolution_clock::now();
  std::cout << "Took " << std::chrono::duration_cast<std::chrono::microseconds>(time_after - start).count() / N_ITER
            << "us/iter" << std::endl;
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}