#ifndef SRC_CRS_PLANNING_CIRCLE_PLANNER_INCLUDE_CIRCLE_PLANNER_PACEJKA_CIRCLE_PLANNER
#define SRC_CRS_PLANNING_CIRCLE_PLANNER_INCLUDE_CIRCLE_PLANNER_PACEJKA_CIRCLE_PLANNER

#include <planning/base_planner.h>
#include <planning/cartesian_reference_point.h>
#include <memory>
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>

namespace crs_planning
{
class PacejkaCirclePlanner : public BasePlanner<cartesian_reference_point, crs_models::pacejka_model::pacejka_car_state>
{
  float angle_ = 0.0;

public:
  PacejkaCirclePlanner(){};

  /**
   * @brief Example, just returns tracking points of a circle
   */
  std::vector<cartesian_reference_point> getPlannedTrajectory(const crs_models::pacejka_model::pacejka_car_state state,
                                                              double timestamp = 0);

  bool goalReached(const crs_models::pacejka_model::pacejka_car_state state,
                   const std::vector<cartesian_reference_point>& trajectory) override;

  /**
  * @brief returns edges of voronoi partitions as vectors
  */
  std::vector<std::vector<double>> getVorEdgesX();
  std::vector<std::vector<double>> getVorEdgesY();
};
}  // namespace crs_planning
#endif /* SRC_CRS_PLANNING_CIRCLE_PLANNER_INCLUDE_CIRCLE_PLANNER_PACEJKA_CIRCLE_PLANNER */
