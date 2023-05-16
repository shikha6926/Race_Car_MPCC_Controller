#ifndef SRC_CRS_DYNAMIC_MODELS_PACEJKA_MODEL_INCLUDE_PACEJKA_MODEL_PACEJKA_DISCRETE
#define SRC_CRS_DYNAMIC_MODELS_PACEJKA_MODEL_INCLUDE_PACEJKA_MODEL_PACEJKA_DISCRETE

#include "pacejka_car_input.h"
#include "pacejka_car_state.h"
#include "pacejka_params.h"
#include "pacejka_params.h"
#include <algorithm>
#include <dynamic_models/continuous_dynamic_model.h>
#include <dynamic_models/discrete_dynamic_model_wrapper.h>
#include <iostream>
#include <iostream>

namespace crs_models
{
namespace pacejka_model
{

typedef Eigen::Matrix<double, 6, 6> StateMatrix;
typedef Eigen::Matrix<double, 6, 2> InputMatrix;
class DiscretePacejkaModel : public DiscreteDynamicModelWrapper<pacejka_car_state, pacejka_car_input, 6, 2>
{
private:
  pacejka_params params_;

public:
  /**
   * @brief Construct a new Discrete Pacejka Model object.
   * matrix
   *
   * @param params Process noise covariance matrix Q. Unit 1/s
   * @param params kinematic parameters
   * @param integration_method integration method. Supported Integrators - see casadi documentation:  cvodes, idas,
   * collocation, oldcollocation, rk
   */
  DiscretePacejkaModel(pacejka_params params, Eigen::Matrix<double, 6, 6> Q, std::string integration_method = "rk");

  /**
   * @brief Construct a new Discrete Pacejka Model object. Process noise covariance matrix Q will default to the unit
   * matrix
   *
   * @param params kinematic parameters
   * @param integration_method integration method. Supported Integrators - see casadi documentation:  cvodes, idas,
   * collocation, oldcollocation, rk
   */
  DiscretePacejkaModel(pacejka_params params, std::string integration_method = "rk")
    : DiscretePacejkaModel(params, Eigen::Matrix<double, 6, 6>::Identity(), integration_method)
  {
    std::cout << "[WARNING] No Q Matrix specified for DiscretePacejkaModel. Using identity Matrix! " << std::endl;
  }

  pacejka_params getParams() const
  {
    return params_;
  }

  /**
   * @brief integrates the state for a given integration_time
   *
   * @param state
   * @param control_input
   * @param integration_time
   * @return pacejka_car_state returns the integrated state
   */
  pacejka_car_state applyModel(const pacejka_car_state state, const pacejka_car_input control_input,
                               double integration_time);
};

}  // namespace pacejka_model

}  // namespace crs_models
#endif /* SRC_CRS_DYNAMIC_MODELS_PACEJKA_MODEL_INCLUDE_PACEJKA_MODEL_PACEJKA_DISCRETE */
