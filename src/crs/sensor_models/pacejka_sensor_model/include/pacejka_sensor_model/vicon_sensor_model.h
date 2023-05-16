#ifndef PACEJKA_SENSOR_MODEL_VICON_SENSOR_MODEL_H
#define PACEJKA_SENSOR_MODEL_VICON_SENSOR_MODEL_H

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <sensor_models/sensor_model.h>

namespace crs_sensor_models
{
namespace pacejka_sensor_models
{
class ViconSensorModel
  : public SensorModel<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input, 6>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  ViconSensorModel() : ViconSensorModel(Eigen::Matrix3d::Identity())  // Measurement dimension is three
  {
    std::cout << "[WARNING] No R Matrix specified for ViconSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the process noise covariance matrix Q yourself.
  ViconSensorModel(const Eigen::Matrix3d& R);  // Measurement dimension is three

  /**
   * @brief Evaluates the measurement model at the given state
   *
   * @param state
   * @return double vector of measurements
   */
  Eigen::Matrix<double, -1, 1> applyModel(const crs_models::pacejka_model::pacejka_car_state& state,
                                          const crs_models::pacejka_model::pacejka_car_input& input) override;

  /**
   * @brief Get the Numerical Jacobian for a given state
   *
   * @param state current state
   * @param H the state jacobian dh/dx
   */
  void getNumericalJacobian(const crs_models::pacejka_model::pacejka_car_state& state,
                            const crs_models::pacejka_model::pacejka_car_input& input,
                            Eigen::Matrix<double, -1, 6>& H) override;

  /**
   * @brief Get the Symbolic Jacobian as casadi function.
   *
   * @return casadi::Function
   */
  casadi::Function getSymbolicJacobian() override;

  static const std::string SENSOR_KEY;

protected:
  casadi::Function measurement_function;
};
}  // namespace pacejka_sensor_models
}  // namespace crs_sensor_models
#endif
