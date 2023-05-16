#ifndef KINEMATIC_MODEL_KINEMATIC_CONTINUOUS_H
#define KINEMATIC_MODEL_KINEMATIC_CONTINUOUS_H

#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include <dynamic_models/continuous_dynamic_model.h>
#include <kinematic_model/kinematic_car_input.h>

#include "kinematic_car_state.h"
#include "kinematic_params.h"

namespace crs_models
{
namespace kinematic_model
{
typedef Eigen::Matrix<double, 4, 4> StateMatrix;
typedef Eigen::Matrix<double, 4, 2> InputMatrix;
class ContinuousKinematicModel : public ContinuousDynamicModel<kinematic_car_state, kinematic_car_input, 4, 2>
{
public:
  ContinuousKinematicModel(kinematic_params params);

  /**
   * @brief applies kinematic model for a given control input
   *
   */
  kinematic_car_state applyModel(const kinematic_car_state state,
                                 const kinematic_model::kinematic_car_input control_input);

  /**
   * @brief Get the Numerical Jacobian for a given state and control input (evaluate symbolic jacobian)
   *
   * @param state current state
   * @param control_input control input
   * @param A the state jacobian df/dx
   * @param B the input jacobian df/du
   */
  void getNumericalJacobian(const kinematic_car_state& state, const kinematic_model::kinematic_car_input& control_input,
                            StateMatrix& A, InputMatrix& B);

  /**
   * @brief Returns the casadi function x_dot = f(x,u)
   *
   * @param state the state x
   * @param control_input  the input u
   * @return
   */
  std::vector<casadi::MX> getContinuousDynamics(const std::vector<casadi::MX> state,
                                                const std::vector<casadi::MX> control_input);

  /**
   * @brief Get the parameters
   *
   * @return kinematic_params
   */
  kinematic_params getParams()
  {
    return params;
  }

private:
  kinematic_params params;
};

}  // namespace kinematic_model

}  // namespace crs_models
#endif
