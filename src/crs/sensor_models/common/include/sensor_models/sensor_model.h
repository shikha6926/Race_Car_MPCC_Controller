#ifndef SENSOR_MODELS_SENSOR_MODEL_H
#define SENSOR_MODELS_SENSOR_MODEL_H

#include <Eigen/Core>
#include <string>

#include <casadi/casadi.hpp>

namespace crs_sensor_models
{
template <typename StateType, typename InputType, int StateDimension>
class SensorModel
{
public:
  SensorModel(int dimension, std::string key) : dimension(dimension), sensor_model_key(key)
  {
  }
  /**
   * @brief Evaluates the measurement model at the given state
   *
   * @param state
   * @param input
   * @return double vector of measruements
   */
  virtual Eigen::Matrix<double, Eigen::Dynamic, 1> applyModel(const StateType& state, const InputType& input) = 0;

  /**
   * @brief Get the Numerical Jacobian for a given state
   *
   * @param state current state
   * @param input current input
   * @param H the state jacobian dh/dx
   */
  virtual void getNumericalJacobian(const StateType& state, const InputType& input,
                                    Eigen::Matrix<double, Eigen::Dynamic, StateDimension>& H) = 0;

  /**
   * @brief Get the Algebraic Jacobian as casadi function.
   *
   * @return casadi::Function
   */
  virtual casadi::Function getSymbolicJacobian() = 0;

  /**
   * @brief Sets the measurement noise covariance matrix
   *
   * @param R
   */
  void setR(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& R)
  {
    R_ = R;
  }

  /**
   * @brief Returns the measurement noise covariance matrix
   *
   * @return const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
   */
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> getR()
  {
    return R_;
  }
  const int dimension;

  /**
   * @brief Returns the key for this sensor model (e.g. vicon, imu, ....)
   *
   * @return std::string
   */
  std::string getKey()
  {
    return sensor_model_key;
  }

protected:
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_;

private:
  std::string sensor_model_key;
};

}  // namespace crs_sensor_models
#endif
