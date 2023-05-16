#ifndef SRC_CRS_CONTROLS_COMMON_INCLUDE_CONTROLS_MODEL_BASED_CONTROLLER
#define SRC_CRS_CONTROLS_COMMON_INCLUDE_CONTROLS_MODEL_BASED_CONTROLLER

#include "base_controller.h"

namespace crs_controls
{
template <typename ModelType, typename StateType, typename InputType>
class ModelBasedController : public BaseController<StateType, InputType>
{
public:
  /**
   * @brief Construct a new Model Based Controller object
   *
   * @param model shared pointer to the underlying model (or config)
   * @param track shared pointer to the track manager
   */
  ModelBasedController(std::shared_ptr<ModelType> model, std::shared_ptr<Trajectory> track)
    : BaseController<StateType, InputType>(track), model_(model){};

  /**
   * @brief Returns the control input for a given measured state.
   *
   * @param state measured state
   * @param timestamp current timestamp in seconds
   * @return InputType control input to apply
   */
  virtual InputType getControlInput(StateType state, double timestamp = 0) = 0;

protected:
  std::shared_ptr<ModelType> model_;
};

}  // namespace crs_controls

#endif /* SRC_CRS_CONTROLS_COMMON_INCLUDE_CONTROLS_MODEL_BASED_CONTROLLER */
