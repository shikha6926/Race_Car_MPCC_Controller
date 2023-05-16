#include <gtest/gtest.h>
#include <casadi/casadi.hpp>
#include <iostream>
#include <random>

#include "pacejka_model/pacejka_car_input.h"
#include "pacejka_model/pacejka_car_state.h"
#include <dynamic_models/utils/data_conversion.h>

#define SUPRESS_OUTPUTS false

TEST(StateTest, struct_creation)
{
  /**
   * @brief Checks if the struct is created correctly (i.e. assigned the right values)
   */

  // Used to create random numbers
  std::random_device rd;
  std::default_random_engine eng(rd());  // NOLINT
  std::uniform_real_distribution<float> distr(0.0, 2.0);

  // Make 20 random  runs.
  for (int i = 0; i < 20; i++)
  {
    // Create random values for struct
    float posx = distr(rd);
    float posy = distr(rd);
    float yawangle = distr(rd);
    float vx = distr(rd);
    float vy = distr(rd);
    float yawrate = distr(rd);

    crs_models::pacejka_model::pacejka_car_state my_state = { posx, posy, yawangle, vx, vy, yawrate };
    // Make sure the random velocities of the struct match
    EXPECT_EQ(posx, my_state.pos_x);
    EXPECT_EQ(posy, my_state.pos_y);
    EXPECT_EQ(yawangle, my_state.yaw);
    EXPECT_EQ(vx, my_state.vel_x);
    EXPECT_EQ(vy, my_state.vel_y);
    EXPECT_EQ(yawrate, my_state.yaw_rate);
  }
}

TEST(StateTest, arithmetics)
{
  /**
   * @brief Checks if the arithmetic operators are correctly implemented (i.e.  outputs anything)
   */

  // Create new struct object
  crs_models::pacejka_model::pacejka_car_state my_state_1 = { 1.0, 1.0, 0.0, 0.5, 0.5, 0.0 };  // define struct my_state
  crs_models::pacejka_model::pacejka_car_state my_state_2 = { 1.0, 1.0, 0.0, 0.5, 0.5, 0.0 };  // define struct my_state

  crs_models::pacejka_model::pacejka_car_state result_sum = { 2.0, 2.0, 0.0, 1.0, 1.0, 0.0 };  // correct result
  crs_models::pacejka_model::pacejka_car_state output_sum = my_state_1 + my_state_2;

  crs_models::pacejka_model::pacejka_car_state result_product = { 2.0, 2.0, 0.0, 1.0, 1.0, 0.0 };  // correct result
  crs_models::pacejka_model::pacejka_car_state output_product = 2 * my_state_1;

  EXPECT_EQ(result_product, output_product);
  EXPECT_EQ(result_sum, output_sum);
  my_state_1 += my_state_2;
  EXPECT_EQ(my_state_1, output_sum);
}

//==============================================================================================================
TEST(InputTest, struct_creation)
{
  /**
   * @brief Checks if the struct is created correctly (i.e. assigned the right values)
   */

  // Used to create random numbers
  std::random_device rd;
  std::default_random_engine eng(rd());  // NOLINT
  std::uniform_real_distribution<float> distr(2.0, 0.0);

  // Make 20 random  runs.
  for (int i = 0; i < 20; i++)
  {
    // Create random values for struct
    float steer_input = distr(rd);
    float torque_input = distr(rd);

    crs_models::pacejka_model::pacejka_car_input my_input = { torque_input, steer_input };
    // Make sure the random inputs of the struct match
    EXPECT_EQ(steer_input, my_input.steer);
    EXPECT_EQ(torque_input, my_input.torque);
  }
}

TEST(InputTest, arithmetics)
{
  /**
   * @brief Checks if the arithmetic operators are correctly implemented (i.e.  outputs anything)
   */

  // Create new struct object
  crs_models::pacejka_model::pacejka_car_input my_input_1 = { 0.5, 0.5 };  // define struct my_input
  crs_models::pacejka_model::pacejka_car_input my_input_2 = { 0.5, 0.5 };  // define struct my_input
  crs_models::pacejka_model::pacejka_car_input result = { 1.0, 1.0 };      // correct result
  crs_models::pacejka_model::pacejka_car_input output = my_input_1 + my_input_2;
  EXPECT_EQ(result, output);
  my_input_1 += my_input_2;
  EXPECT_EQ(my_input_1, output);
}

/**
 * @brief Checks that the conversion from state to vector works as expected
 */
TEST(DataConversion, convert_to_vectors)
{
  crs_models::pacejka_model::pacejka_car_state state = { 1.0, 1.0, 0.0, 0.5, 0.5, 0.0 };  // define struct state
  crs_models::pacejka_model::pacejka_car_state state_to_add = {
    1.0, 1.0, 0.0, 0.5, 0.5, 0.0
  };  // define struct that will be added to state for tests

  crs_models::pacejka_model::pacejka_car_input input = { 0.5, 0.5 };         // define struct input
  crs_models::pacejka_model::pacejka_car_input input_to_add = { 0.5, 0.5 };  // define struct that will be added to
                                                                             // input

  // Check Read only
  auto state_const_vector = commons::convertToConstVector(state);
  EXPECT_EQ(*state_const_vector[0], state.pos_x);
  EXPECT_EQ(*state_const_vector[1], state.pos_y);
  EXPECT_EQ(*state_const_vector[2], state.yaw);
  EXPECT_EQ(*state_const_vector[3], state.vel_x);
  EXPECT_EQ(*state_const_vector[4], state.vel_y);
  EXPECT_EQ(*state_const_vector[5], state.yaw_rate);

  auto input_const_vector = commons::convertToConstVector(input);
  EXPECT_EQ(*input_const_vector[0], input.torque);
  EXPECT_EQ(*input_const_vector[1], input.steer);

  auto state_to_modify = state;
  // Check assigment
  auto state_writable_vector = commons::convertToVector(state_to_modify);
  (*state_writable_vector[0]) += state_to_add.pos_x;
  (*state_writable_vector[1]) += state_to_add.pos_y;
  (*state_writable_vector[2]) += state_to_add.yaw;
  (*state_writable_vector[3]) += state_to_add.vel_x;
  (*state_writable_vector[4]) += state_to_add.vel_y;
  (*state_writable_vector[5]) += state_to_add.yaw_rate;
  EXPECT_EQ(state_to_modify, state + state_to_add);

  auto input_to_modify = input;
  auto input_writable_vector = commons::convertToVector(input_to_modify);
  (*input_writable_vector[0]) += input_to_add.torque;
  (*input_writable_vector[1]) += input_to_add.steer;
  EXPECT_EQ(input_to_modify, input + input_to_add);
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}