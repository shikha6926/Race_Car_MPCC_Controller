#ifndef PACEJKA_MODEL_TESTS_TEST_UTILS_H
#define PACEJKA_MODEL_TESTS_TEST_UTILS_H
#include <random>

#include "pacejka_model/pacejka_car_input.h"
#include "pacejka_model/pacejka_car_state.h"
#include "pacejka_model/pacejka_params.h"
#include <dynamic_models/utils/data_conversion.h>
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

void loadRandomPacejkaParams(crs_models::pacejka_model::pacejka_params& params, const std::string path)
{
  crs_models::pacejka_model::loadParamsFromFile(path, params);

  float rel_diff = 0.1;  // 10 percent difference
  params.lr *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.lf *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.m *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.I *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Df *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cf *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Bf *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Dr *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cr *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Br *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cm1 *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cm2 *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Cd *= 1 + getRandomFloat(-rel_diff, rel_diff);
  params.Croll *= 1 + getRandomFloat(-rel_diff, rel_diff);
}

void loadRandomState(crs_models::pacejka_model::pacejka_car_state& state)
{
  state.pos_x = getRandomFloat(-10, 10);
  state.pos_y = getRandomFloat(-10, 10);
  state.yaw = getRandomFloat(-3.1415, 3.1415);
  state.vel_x = getRandomFloat(0.5, 1);
  state.vel_y = getRandomFloat(-0.5, 0.5);
  state.yaw_rate = getRandomFloat(-0.5, 0.5);
}

void loadRandomInput(crs_models::pacejka_model::pacejka_car_input& input)
{
  input.torque = getRandomFloat(0.1, 1);
  input.steer = getRandomFloat(-0.4, 0.4);
}

#endif  // PACEJKA_MODEL_TESTS_TEST_UTILS_H
