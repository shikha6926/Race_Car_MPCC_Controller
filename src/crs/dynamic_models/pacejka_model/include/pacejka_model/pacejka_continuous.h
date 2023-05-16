#ifndef PACEJKA_MODEL_PACEJKA_CONTINUOUS_H
#define PACEJKA_MODEL_PACEJKA_CONTINUOUS_H

#include "pacejka_car_input.h"
#include "pacejka_car_state.h"
#include "pacejka_params.h"
#include "pacejka_params.h"
#include <algorithm>
#include <dynamic_models/continuous_dynamic_model.h>
#include <iostream>
#include <iostream>

namespace crs_models
{
namespace pacejka_model
{
typedef Eigen::Matrix<double, 6, 6> StateMatrix;
typedef Eigen::Matrix<double, 6, 2> InputMatrix;

class ContinuousPacejkaModel : public ContinuousDynamicModel<pacejka_car_state, pacejka_car_input, 6, 2>
{
public:
  ContinuousPacejkaModel(pacejka_params params);  // Constructor

  /**
   * @brief applies Pacejka model for a given control input
   *
   */
  pacejka_car_state applyModel(const pacejka_car_state state, const pacejka_car_input control_input);

  /**
   * @brief Get the Numerical Jacobian for a given state and control input (evaluate symbolic jacobian)
   *
   * @param state current state
   * @param control_input control input
   * @param A the state jacobian df/dx
   * @param B the input jacobian df/du
   */
  void getNumericalJacobian(const pacejka_car_state& state, const pacejka_car_input& control_input, StateMatrix& A,
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
  pacejka_params params;
};

}  // namespace pacejka_model

}  // namespace crs_models
#endif
