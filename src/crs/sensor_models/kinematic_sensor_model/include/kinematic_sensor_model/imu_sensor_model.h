#ifndef KINEMATIC_SENSOR_MODEL_IMU_SENSOR_MODEL_H
#define KINEMATIC_SENSOR_MODEL_IMU_SENSOR_MODEL_H

#include <sensor_models/sensor_model.h>

#include <kinematic_model/kinematic_car_input.h>
#include <kinematic_model/kinematic_car_state.h>
#include <kinematic_model/kinematic_continuous.h>

namespace crs_sensor_models
{
namespace kinematic_sensor_models
{
/**
 * @brief Creates a sensor model that is used to measure linear accelarations and angular velocities
 *
 */
class ImuSensorModel : public SensorModel<crs_models::kinematic_model::kinematic_car_state,
                                          crs_models::kinematic_model::kinematic_car_input, 4>
{
public:
  // Option 1: Create an object with no process noise covariance matrix Q defined. Will use identity.
  /**
   * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the pacejka state and
   * therefore the continuous model is needed
   *
   * @param kinematic_cont the continuous model
   */
  ImuSensorModel(const std::shared_ptr<crs_models::kinematic_model::ContinuousKinematicModel> kinematic_cont)
    : ImuSensorModel(kinematic_cont, Eigen::Matrix3d::Identity())  // Measurement dimension is three
  {
    std::cout << "[WARNING] No R Matrix specified for ImuSensorModel. Using identity Matrix! " << std::endl;
  }

  // Option 2: Create an object and specify the process noise covariance matrix Q yourself.
  /**
   * @brief Construct a new Imu Sensor Model. Note that the accelerations are not part of the pacejka state and
   * therefore the continuous model is needed
   *
   * @param kinematic_cont the continuous model
   * @param R measurement covariance Matrix
   */
  ImuSensorModel(const std::shared_ptr<crs_models::kinematic_model::ContinuousKinematicModel> kinematic_cont,
                 const Eigen::Matrix3d& R);  // Measurement dimension is three

  /**
   * @brief Evaluates the measurement model at the given state
   *
   * @param state
   * @return double vector of measurements
   */
  Eigen::Matrix<double, -1, 1> applyModel(const crs_models::kinematic_model::kinematic_car_state& state,
                                          const crs_models::kinematic_model::kinematic_car_input& input) override;

  /**
   * @brief Get the Numerical Jacobian for a given state
   *
   * @param state current state
   * @param H the state jacobian dh/dx
   */
  void getNumericalJacobian(const crs_models::kinematic_model::kinematic_car_state& state,
                            const crs_models::kinematic_model::kinematic_car_input& input,
                            Eigen::Matrix<double, -1, 4>& H) override;

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
}  // namespace kinematic_sensor_models
}  // namespace crs_sensor_models
#endif
