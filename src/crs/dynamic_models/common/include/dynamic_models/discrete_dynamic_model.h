#ifndef SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_DISCRETE_DYNAMIC_MODEL
#define SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_DISCRETE_DYNAMIC_MODEL
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>

namespace crs_models
{
template <typename StateType, typename InputType, int StateDimension, int InputDimension>
class DiscreteDynamicModel
{
public:
  DiscreteDynamicModel(Eigen::Matrix<double, StateDimension, StateDimension> Q) : Q_(Q){};

  /**
   * @brief Calculates the state of the system after evolving for a certain integration_time
   *
   * @param state starting state
   * @param control_input input to the system
   * @param integration_time timestep to integrate
   * @param output_state the output state
   */
  virtual StateType applyModel(const StateType state, const InputType control_input, double integration_time) = 0;

  /**
   * @brief Gets the jacobian of the discrete system
   *
   * @param state starting state
   * @param control_input control input that should be applied
   * @param integration_time integration_time
   * @param A df/dx of the discrete system
   * @param B df/du of the discrete system
   */
  virtual void getJacobian(const StateType& state, const InputType& control_input, double integration_time,
                           Eigen::Matrix<double, StateDimension, StateDimension>& A,
                           Eigen::Matrix<double, StateDimension, InputDimension>& B) = 0;
  /**
   * @brief Sets the Process Noise Covariance Matrix associated with these dynamics.
   * Note that the Unit of Q is 1/s.
   *
   * @param Q Process Noise Covariance Matrix
   */
  void setQ(const Eigen::Matrix<double, StateDimension, StateDimension>& Q)
  {
    Q_ = Q;
  }

  /**
   * @brief Returns the Process Noise Covariance Matrix associated with these dynamics.
   * Note that the Unit of Q is 1/s.
   *
   * @return const Eigen::Matrix<double, StateDimension, StateDimension>
   */
  const Eigen::Matrix<double, StateDimension, StateDimension> getQ()
  {
    return Q_;
  }

private:
  Eigen::Matrix<double, StateDimension, StateDimension> Q_;
};
}  // namespace crs_models
#endif /* SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_DISCRETE_DYNAMIC_MODEL */
