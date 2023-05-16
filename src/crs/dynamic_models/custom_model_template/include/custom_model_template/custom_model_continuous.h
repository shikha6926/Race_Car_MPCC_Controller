#ifndef CUSTOM_MODEL_TEMPLATE_CUSTOM_MODEL_CONTINUOUS_H
#define CUSTOM_MODEL_TEMPLATE_CUSTOM_MODEL_CONTINUOUS_H

#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "custom_model_template/custom_input.h"
#include "custom_model_template/custom_params.h"
#include "custom_model_template/custom_state.h"
#include <dynamic_models/continuous_dynamic_model.h>

// State dimension is 6, Input dimenstion is 2
typedef Eigen::Matrix<double, 6, 6> StateMatrix;
typedef Eigen::Matrix<double, 6, 2> InputMatrix;

namespace crs_models
{
namespace custom_model
{
class ContinuousCustomModel : public ContinuousDynamicModel<custom_state, custom_input, 6, 2>
{
public:
  ContinuousCustomModel(custom_params params);  // Constructor

  /**
   * @brief applies the model for a given control input
   *
   */
  custom_state applyModel(const custom_state state, const custom_input control_input);

  /**
   * @brief Get the Numerical Jacobian for a given state and control input (evaluate symbolic jacobian)
   *
   * @param state current state
   * @param control_input control input
   * @param A the state jacobian df/dx
   * @param B the input jacobian df/du
   */
  void getNumericalJacobian(const custom_state& state, const custom_input& control_input, StateMatrix& A,
                            InputMatrix& B);

  /**
   * @brief Returns the casadi function x_dot = f(x,u)
   *
   * @param state the state x
   * @param control_input  the input u
   * @return
   */
  std::vector<casadi::MX> getContinuousDynamics(const std::vector<casadi::MX> state,
                                                const std::vector<casadi::MX> control_input);

private:
  custom_params params;
};

}  // namespace custom_model

}  // namespace crs_models
#endif
