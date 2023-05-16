#ifndef CONTROLS_MPC_CONTROLLER_H
#define CONTROLS_MPC_CONTROLLER_H

#include "model_based_controller.h"

namespace crs_controls
{
template <typename ModelType, typename StateType, typename InputType>
class MpcController : public ModelBasedController<ModelType, StateType, InputType>
{
public:
  /**
   * @brief Construct a new Model Based Controller object
   *
   * @param model shared pointer to the underlying model (or config)
   * @param track shared pointer to the track manager
   */
  MpcController(std::shared_ptr<ModelType> model, std::shared_ptr<Trajectory> track)
    : ModelBasedController<ModelType, StateType, InputType>(model, track){};

  /**
   * @brief Returns the control input for a given measured state.
   *
   * @param state measured state
   * @param timestamp current timestamp in seconds
   * @return InputType control input to apply
   */
  virtual InputType getControlInput(StateType state, double timestamp = 0) = 0;

  virtual std::vector<std::vector<double>> getPlannedTrajectory() = 0;
};

}  // namespace crs_controls

#endif /* CONTROLS_MPC_CONTROLLER_H */
