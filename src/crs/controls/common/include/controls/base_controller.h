#ifndef SRC_CRS_CONTROLS_COMMON_INCLUDE_CONTROLS_BASE_CONTROLLER
#define SRC_CRS_CONTROLS_COMMON_INCLUDE_CONTROLS_BASE_CONTROLLER

#include <commons/trajectory.h>
#include <memory>

namespace crs_controls
{
template <typename StateType, typename InputType>
class BaseController
{
public:
  /**
   * @brief Construct a new Base Controller object
   *
   * @param trajectory the trajectory manager that should be used by this controller
   */
  BaseController(std::shared_ptr<Trajectory> trajectory) : trajectory_(trajectory){};

  /**
   * @brief Returns the control input for a given measured state.
   *
   * @param state measured state
   * @param timestamp current timestamp in seconds
   * @return InputType control input to apply
   */
  virtual InputType getControlInput(StateType state, double timestamp = 0) = 0;

  /**
   * @brief Returns the trajectory manager associated with this controller
   *
   * @return const std::shared_ptr<const Trajectory>
   */
  template <typename TrajectoryType = Trajectory>
  const std::shared_ptr<TrajectoryType> getTrajectory() const
  {
    return std::static_pointer_cast<TrajectoryType>(trajectory_);
  }

  /**
   * @brief Returns wether the controller is initialized
   *
   * @return true if controller is already initialized
   * @return false if controller is not initialized
   */
  virtual const bool isInitializing() = 0;

protected:
  std::shared_ptr<Trajectory> trajectory_;
};

}  // namespace crs_controls

#endif /* SRC_CRS_CONTROLS_COMMON_INCLUDE_CONTROLS_BASE_CONTROLLER */
