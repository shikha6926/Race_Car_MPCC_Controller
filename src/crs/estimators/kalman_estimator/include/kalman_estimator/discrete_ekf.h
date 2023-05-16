#ifndef SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR_DISCRETE_EKF
#define SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR_DISCRETE_EKF

#include <memory>

#include <dynamic_models/discrete_dynamic_model.h>
#include <dynamic_models/utils/data_conversion.h>
#include <sensor_models/sensor_measurement.h>
#include <sensor_models/sensor_model.h>

#include <Eigen/Dense>
#include <stdexcept>

#include <estimators/model_based_estimator.h>

namespace crs_estimators
{
namespace kalman
{
template <typename StateType, typename InputType, int StateDimension, int InputDimension>
class DiscreteEKF : public ModelBasedEstimator<StateType, InputType>
{
  typedef std::shared_ptr<crs_models::DiscreteDynamicModel<StateType, InputType, StateDimension, InputDimension>>
      discrete_model_ptr;

public:
  // Constructor
  DiscreteEKF(discrete_model_ptr discrete_model, StateType initial_state, InputType initial_input,
              Eigen::Matrix<double, StateDimension, StateDimension> P_init)
    : discrete_model(discrete_model)
    , x_prior_(initial_state)
    , x_posterior_(initial_state)
    , previous_input_(initial_input)
    , best_state_(initial_state)
    , P_prior(P_init)
    , P_posterior(P_init){};

  // Constructor
  DiscreteEKF(discrete_model_ptr discrete_model, StateType initial_state,
              Eigen::Matrix<double, StateDimension, StateDimension> P_init)
    : discrete_model(discrete_model)
    , x_prior_(initial_state)
    , x_posterior_(initial_state)
    , best_state_(initial_state)
    , P_prior(P_init)
    , P_posterior(P_init){};

  /**
   * @brief Function that gets called whenever a new input is applied.
   *        This internally calls the prediction step of the EKF with the older input and total duration it was applied
   * for.
   *
   * @param input e.g. control input
   * @param timestamp current time in s when this input was applied
   */
  void controlInputCallback(const InputType input, const double timestamp) override
  {
    if (last_valid_ts_ != -1)  // There was an older input available which was received at time last_valid_ts_ / priors
                               // exist
    {
      double timestep = timestamp - last_valid_ts_;  // how long the previous input was applied (= how long we need to
                                                     // propogate the state)
      if (timestep < 0)
        return;
      predict(previous_input_, timestep);  // propogate the state through model / model prediction of state
    }

    previous_input_ = input;
    last_valid_ts_ = timestamp;
  }

  /**
   * @brief Function that gets called whenever a new measurement is received.
   *        This internally calls the measurement update step of the EKF.
   *        The predict step is called with the previous input,
   *        in order to make sure the prior state is propogated forward to the same timestamp as the received
   * measurement.
   *
   * @param measurement
   */
  void measurementCallback(const crs_sensor_models::measurement measurement) override
  {
    // Update prior state to current time (time the measurement is available)
    controlInputCallback(previous_input_, measurement.timestamp);
    // Apply posterior update
    measurementUpdate(measurement);
  }

  /**
   * @brief This is the prediction step of the EKF. The state is predicated based on the model.
   *        This function only updates the prior state of the ekf
   *
   * @param input e.g. control input
   * @param timestep how long to apply the model for (how long to integrate for)
   */
  void predict(const InputType& input, const double timestep)
  {
    // F and B are the martices that will be filled with the jacobian values. F = df/dx, B = df/du
    Eigen::Matrix<double, StateDimension, StateDimension> F =
        Eigen::Matrix<double, StateDimension, StateDimension>::Zero();
    Eigen::Matrix<double, StateDimension, InputDimension> B =
        Eigen::Matrix<double, StateDimension, InputDimension>::Zero();
    discrete_model->getJacobian(x_prior_, input, timestep, F, B);

    auto x_predicted_ = discrete_model->applyModel(x_prior_, input, timestep);

    if (!commons::convertToEigen(x_predicted_).allFinite())
    {
      std::cout << "[WARN] NaN values detected in predict step. " << std::endl;
      // Do not update anything to not publish NaN values
      return;
    }
    x_prior_ = x_predicted_;
    best_state_ = x_prior_;

    P_prior = F * P_prior * F.transpose() + timestep * discrete_model->getQ();
  }

  /**
   * @brief This is the measurement update step of the EKF.
   *        The state prediction based on the model is updated using measurement inofrmation.
   *        This function updates the prior and posterior state of the ekf
   *
   * @param data measurement information, e.g. type of sensor, measurement data
   */
  void measurementUpdate(const crs_sensor_models::measurement& data)
  {
    // Get sensor model for key with validity checks
    auto entry = key_to_sensor_model_.find(data.sensor_key);  // entry is pair containing (key, sensor_model)
    if (entry == key_to_sensor_model_.end())
    {  // Sensor key did not exist
      throw std::invalid_argument("measurementUpdate. No SensorModel found for given key: " + data.sensor_key);
    }
    std::shared_ptr<crs_sensor_models::SensorModel<StateType, InputType, StateDimension>> sensor_model = entry->second;
    // Start real measurement update
    // H is the martices that will be filled with the jacobian values. H = df_sensor/dx
    Eigen::Matrix<double, -1, StateDimension> H =
        Eigen::Matrix<double, -1, StateDimension>::Zero(sensor_model->dimension, StateDimension);
    sensor_model->getNumericalJacobian(x_prior_, previous_input_, H);
    Eigen::Matrix<double, -1, 1> z_hat = sensor_model->applyModel(x_prior_, previous_input_);

    auto y_hat = (data.measurement_data - z_hat);
    // // Update P. We use the Joseph form to avoid numerical issues, particularly with the Pacejka model.
    Eigen::Matrix<double, -1, -1> S = H * P_prior * H.transpose() + sensor_model->getR();
    auto K = P_prior * H.transpose() * S.inverse();

    Eigen::Matrix<double, StateDimension, StateDimension> I_KH =
        Eigen::MatrixXd::Identity(StateDimension, StateDimension) - K * H;

    x_posterior_ = commons::convertToState<StateType, -1>(commons::convertToEigen(x_prior_) + K * y_hat);
    if (!commons::convertToEigen(x_posterior_).allFinite())
    {
      std::cout << "[WARN] NaN values detected in measurement udpate step. " << std::endl;
      // Do not update state to not introduce nan values
      return;
    }
    P_posterior = I_KH * P_prior * I_KH.transpose() + K * sensor_model->getR() * K.transpose();  // more stable

    P_prior = P_posterior;

    x_prior_ = x_posterior_;
    best_state_ = x_posterior_;
  }

  StateType getStateEstimate() const override
  {
    // return the current state estimate
    return best_state_;
  }

  InputType getLastInput() const override
  {
    return previous_input_;
  }

  /**
   * @brief Adds a sensor model to the map (key and sensor_model)
   *
   * @param sensor_key
   * @param sensors_model
   */
  void
  addSensorModel(std::string sensor_key,
                 std::shared_ptr<crs_sensor_models::SensorModel<StateType, InputType, StateDimension>> sensors_model)
  {
    key_to_sensor_model_.insert(std::make_pair<>(sensor_key, sensors_model));
  }

  void setState(StateType state)
  {
    x_prior_ = state;
    x_posterior_ = state;
    best_state_ = state;
  }

  double getLastValidTs() const override
  {
    return last_valid_ts_;
  }

  // map saves objects that have a key and a value (here key = string, e.g. vicon, value = sensor_model e.g.
  // vicon_sensor_model)
  std::map<std::string, std::shared_ptr<crs_sensor_models::SensorModel<StateType, InputType, StateDimension>>>
      key_to_sensor_model_;

private:
  /**
   * @brief Prior refers to before the measurement update step of the EKF is done.
   *        It is based soley on the forward dynamic predictions.
   */
  StateType x_prior_;
  Eigen::Matrix<double, StateDimension, StateDimension> P_prior;
  Eigen::Matrix<double, StateDimension, StateDimension> P_posterior;
  /**
   * @brief  Posterior refers to after the measurement update step of the EKF is done.
   *
   */
  StateType x_posterior_;

  StateType best_state_;
  InputType previous_input_;

  double last_valid_ts_ = -1;

  discrete_model_ptr discrete_model;
};
}  // namespace kalman
}  // namespace crs_estimators
#endif /* SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR_DISCRETE_EKF */
