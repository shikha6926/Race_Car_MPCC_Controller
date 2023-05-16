#ifndef KINEMATIC_MODEL_TESTS_TEST_UTILS_H
#define KINEMATIC_MODEL_TESTS_TEST_UTILS_H
#include <random>

#include "kinematic_model/kinematic_car_input.h"
#include "kinematic_model/kinematic_car_state.h"
#include "kinematic_model/kinematic_params.h"
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

// For random number generation
std::random_device rd;
std::default_random_engine eng(rd());  // NOLINT
std::uniform_real_distribution<float> distr(0.0, 1.0);

float getRandomFloat(float lower_limit, float upper_limit)
{
  return (distr(rd) * (upper_limit - lower_limit)) + lower_limit;
}

void loadRandomKinematicParams(crs_models::kinematic_model::kinematic_params& params, const std::string path)
{
  // TODO(sabodmer) find cleaner solution
  crs_models::kinematic_model::loadParamsFromFile(path, params);

  float rel_diff = 0.1;  // 10 percent difference
  params.lr *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.lf *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.tau *= 1 + getRandomFloat(-rel_diff, rel_diff);
}

void loadRandomState(crs_models::kinematic_model::kinematic_car_state& state)
{
  state.pos_x = getRandomFloat(-10, 10);
  state.pos_y = getRandomFloat(-10, 10);
  state.yaw = getRandomFloat(-3.1415, 3.1415);
  state.velocity = getRandomFloat(0.5, 1);
}

void loadRandomInput(crs_models::kinematic_model::kinematic_car_input& input)
{
  input.torque = getRandomFloat(0.1, 1);
  input.steer = getRandomFloat(-0.5, 0.5);
}

#endif  // KINEMATIC_MODEL_TESTS_TEST_UTILS_H
