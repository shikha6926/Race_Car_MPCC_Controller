#ifndef SRC_CRS_PLANNING_COMMON_INCLUDE_CONTROLS_BASE_PLANNER
#define SRC_CRS_PLANNING_COMMON_INCLUDE_CONTROLS_BASE_PLANNER

#include <commons/static_track_trajectory.h>
#include <memory>

namespace crs_planning
{
template <typename TrajectoryType, typename StateType>
class BasePlanner
{
public:
  /**
   * @brief Construct a new Base Planner object
   *
   */
  BasePlanner(){};

  /**
   * @brief Returns the future trajectory for the current state.
   *
   * @param state measured state
   * @param timestamp current timestamp in seconds
   * @return InputType control input to apply
   */
  virtual std::vector<TrajectoryType> getPlannedTrajectory(const StateType state, double timestamp = 0) = 0;

  virtual std::vector<std::vector<double>> getVorEdgesX() = 0;

  virtual std::vector<std::vector<double>> getVorEdgesY() = 0;

  virtual bool goalReached(const StateType state, const std::vector<TrajectoryType>& trajectory) = 0;
};

}  // namespace crs_planning

#endif /* SRC_CRS_PLANNING_COMMON_INCLUDE_CONTROLS_BASE_PLANNER */
