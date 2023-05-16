#include <gtest/gtest.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>
#include <math.h>
#include <random>

#include "kinematic_model/kinematic_car_input.h"

#include "kinematic_model/kinematic_car_state.h"
#include "kinematic_model/kinematic_continuous.h"
#include "kinematic_model/kinematic_discrete.h"
#include "kinematic_model/kinematic_params.h"
#include "kinematic_model/tests/test_utils.h"
#include <dynamic_models/utils/data_conversion.h>
#include <experimental/filesystem>

#define N_ITER 100

/**
 * @brief Checks if continuous model can be called
 */
TEST(kinematicTests, testContModel)
{
  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::kinematic_model::kinematic_params params;
    loadRandomKinematicParams(params, "params/example_kinematic_params.yaml");
    crs_models::kinematic_model::ContinuousKinematicModel model(params);
    // only has a position and forward velocity
    crs_models::kinematic_model::kinematic_car_state current_state;
    loadRandomState(current_state);

    crs_models::kinematic_model::kinematic_car_input current_input;
    loadRandomInput(current_input);

    for (int step = 0; step < 5; step++)
    {
      auto next_state = model.applyModel(current_state, current_input);
      // Move current state
      current_state = next_state;
    }
  }
}

/**
 * @brief Checks if the discrete system drives forward
 *        Tests the casadi integrator by calling applyModel()
 */
TEST(kinematicTests, testDiscreteDriveForward)
{
  for (int iteration_counter = 0; iteration_counter < N_ITER; iteration_counter++)
  {
    crs_models::kinematic_model::kinematic_params params;
    loadRandomKinematicParams(params, "params/example_kinematic_params.yaml");
    crs_models::kinematic_model::DiscreteKinematicModel model(params);
    // only has a position and forward velocity
    crs_models::kinematic_model::kinematic_car_state current_state = { getRandomFloat(-5, 5), getRandomFloat(-5, 5),
                                                                       0.0, getRandomFloat(0.4, 2) };
    crs_models::kinematic_model::kinematic_car_input current_input = { getRandomFloat(0.3, 1),
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

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}