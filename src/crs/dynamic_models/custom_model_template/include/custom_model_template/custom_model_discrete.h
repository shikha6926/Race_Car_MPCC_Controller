#ifndef CUSTOM_MODEL_TEMPLATE_CUSTOM_MODEL_DISCRETE_H
#define CUSTOM_MODEL_TEMPLATE_CUSTOM_MODEL_DISCRETE_H

#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "custom_model_template/custom_model_continuous.h"
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <dynamic_models/utils/data_conversion.h>

namespace crs_models
{
namespace custom_model
{
// ==================== TODO change state dimensions ====================
class DiscreteCustomModel : public DiscreteDynamicModelWrapper<custom_state, custom_input, 6, 2>
{
public:
  // Option 1: Create an object and specify the process noise covariance matrix Q yourself.
  // ==================== TODO change state dimensions ====================
  DiscreteCustomModel(custom_params params, Eigen::Matrix<double, 6, 6> Q);

  // Option 2: Create an object with no process noise covariance matrix Q defined. Will use identity.
  // ==================== TODO change state dimensions ====================
  DiscreteCustomModel(custom_params params) : DiscreteCustomModel(params, Eigen::Matrix<double, 6, 6>::Identity())
  {
    std::cout << "[WARNING] No Q Matrix specified for DiscreteCustomModel. Using identity Matrix! " << std::endl;
  }

  /**
   * @brief integrates the state for a given integration_time
   *
   * @param state
   * @param control_input
   * @param integration_time
   * @return custom_state returns the integrated state
   */
  custom_state applyModel(const custom_state state, const custom_input control_input, double integration_time);
};

}  // namespace custom_model

}  // namespace crs_models
#endif
