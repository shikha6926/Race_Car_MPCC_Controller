#ifndef DYNAMIC_MODELS_DISCRETE_DYNAMIC_MODEL_WRAPPER_H
#define DYNAMIC_MODELS_DISCRETE_DYNAMIC_MODEL_WRAPPER_H

#include "continuous_dynamic_model.h"
#include "discrete_dynamic_model.h"
#include <algorithm>
#include <casadi/casadi.hpp>
#include <casadi/casadi.hpp>
#include <iostream>

namespace crs_models
{
template <typename StateType, typename InputType, int StateDimension, int InputDimension>
class DiscreteDynamicModelWrapper : public DiscreteDynamicModel<StateType, InputType, StateDimension, InputDimension>
{
protected:
  std::unique_ptr<crs_models::ContinuousDynamicModel<StateType, InputType, StateDimension, InputDimension>> cont_model;
  casadi::Function integrator_;

public:
  DiscreteDynamicModelWrapper(Eigen::Matrix<double, StateDimension, StateDimension> Q)
    : DiscreteDynamicModel<StateType, InputType, StateDimension, InputDimension>(Q){};

  /**
   * @brief Calculates the state of the system after evolving for a certain integration_time
   *
   * @param state starting state
   * @param control_input input to the system
   * @param integration_time timestap to integrate
   * @param output_state the output state
   */
  virtual StateType applyModel(const StateType state, const InputType control_input, double integration_time) = 0;

  /**
   * @brief Gets the jacobian of the discrete system using rk4
   *
   * @param state starting state
   * @param control_input control input that should be applied
   * @param integration_time integration_time
   * @param A df/dx of the discrete system
   * @param B df/du of the discrete system
   */
  void getJacobian(const StateType& state, const InputType& control_input, double integration_time,
                   Eigen::Matrix<double, StateDimension, StateDimension>& F,
                   Eigen::Matrix<double, StateDimension, InputDimension>& D)
  {
    StateType k1 = cont_model->applyModel(state, control_input);
    StateType k2 = cont_model->applyModel(state + (integration_time / 2.0) * k1, control_input);
    StateType k3 = cont_model->applyModel(state + (integration_time / 2.0) * k2, control_input);
    StateType k4 = cont_model->applyModel(state + integration_time * k3, control_input);
    StateType next_state = state + (integration_time / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

    Eigen::Matrix<double, StateDimension, StateDimension> F_k1;
    Eigen::Matrix<double, StateDimension, InputDimension> D_k1;
    cont_model->getNumericalJacobian(state, control_input, F_k1, D_k1);

    Eigen::Matrix<double, StateDimension, StateDimension> F_k2;
    Eigen::Matrix<double, StateDimension, InputDimension> D_k2;
    cont_model->getNumericalJacobian(state + (integration_time / 2.0) * k1, control_input, F_k2, D_k2);
    F_k2 = F_k2 * (Eigen::Matrix<double, StateDimension, StateDimension>::Identity() + integration_time * F_k1 / 2.0);

    Eigen::Matrix<double, StateDimension, StateDimension> F_k3;
    Eigen::Matrix<double, StateDimension, InputDimension> D_k3;
    cont_model->getNumericalJacobian(state + (integration_time / 2.0) * k2, control_input, F_k3, D_k3);
    F_k3 = F_k3 * (Eigen::Matrix<double, StateDimension, StateDimension>::Identity() + integration_time * F_k2 / 2.0);

    Eigen::Matrix<double, StateDimension, StateDimension> F_k4;
    Eigen::Matrix<double, StateDimension, InputDimension> D_k4;
    cont_model->getNumericalJacobian(state + integration_time * k3, control_input, F_k4, D_k4);

    F_k4 = F_k4 * (Eigen::Matrix<double, StateDimension, StateDimension>::Identity() + integration_time * F_k3);

    F = Eigen::Matrix<double, StateDimension, StateDimension>::Identity() +
        (integration_time / 6.0) * (F_k1 + 2 * F_k2 + 2 * F_k3 + F_k4);
    D = Eigen::Matrix<double, StateDimension, InputDimension>::Identity() +
        (integration_time / 6.0) * (D_k1 + 2 * D_k1 + 2 * D_k3 + D_k4);
  }
};
};  // namespace crs_models
#endif
