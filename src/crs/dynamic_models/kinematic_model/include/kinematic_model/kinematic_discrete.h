#ifndef KINEMATIC_MODEL_KINEMATIC_DISCRETE_H
#define KINEMATIC_MODEL_KINEMATIC_DISCRETE_H

#include <Eigen/Core>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <iostream>

#include "kinematic_car_state.h"
#include "kinematic_continuous.h"
#include "kinematic_params.h"
#include <dynamic_models/continuous_dynamic_model.h>
#include <dynamic_models/discrete_dynamic_model_wrapper.h>

namespace crs_models
{
namespace kinematic_model
{

class DiscreteKinematicModel
  : public DiscreteDynamicModelWrapper<kinematic_car_state, kinematic_model::kinematic_car_input, 4, 2>
{
public:
  /**
   * @brief Construct a new Discrete Kinematic Model object.
   *
   * @param Q Process noise covariance matrix. Unit 1/s
   * @param params kinematic parameters
   * @param integration_method integration method. Supported Integrators - see casadi documentation:  cvodes, idas,
   * collocation, oldcollocation, rk
   */
  DiscreteKinematicModel(kinematic_params params, Eigen::Matrix<double, 4, 4> Q, std::string integration_method = "rk");

  /**
   * @brief Construct a new Discrete Kinematic Model object. Process noise covariance matrix Q will default to the unit
   * matrix
   *
   * @param params kinematic parameters
   * @param integration_method integration method. Supported Integrators - see casadi documentation:  cvodes, idas,
   * collocation, oldcollocation, rk
   */
  DiscreteKinematicModel(kinematic_params params)
    : DiscreteKinematicModel(params, Eigen::Matrix<double, 4, 4>::Identity())
  {
    std::cout << "[WARNING] No Q Matrix specified for DiscreteKinematicModel. Using identity Matrix! " << std::endl;
  }

  /**
   * @brief integrates the state for a given integration_time
   *
   * @param state
   * @param control_input
   * @param integration_time
   * @return kinematic_car_state returns the integrated state
   */
  kinematic_car_state applyModel(const kinematic_car_state state,
                                 const kinematic_model::kinematic_car_input control_input, double integration_time);
};

}  // namespace kinematic_model

}  // namespace crs_models
#endif
