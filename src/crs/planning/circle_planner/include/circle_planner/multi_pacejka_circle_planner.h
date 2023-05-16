#ifndef SRC_CRS_PLANNING_CIRCLE_PLANNER_INCLUDE_CIRCLE_PLANNER_MULTI_PACEJKA_CIRCLE_PLANNER
#define SRC_CRS_PLANNING_CIRCLE_PLANNER_INCLUDE_CIRCLE_PLANNER_MULTI_PACEJKA_CIRCLE_PLANNER

#include <planning/base_planner.h>
#include <planning/multi_car_cartesian_reference_point.h>
#include <memory>
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <map>

namespace crs_planning
{
class MultiPacejkaCirclePlanner
  : public BasePlanner<multi_car_cartesian_reference_point,
                       std::map<std::string, crs_models::pacejka_model::pacejka_car_state>>
{
private:
  double angle_ = 0.0;
  std::map<std::string, bool> car_to_reach_state_;

public:
  MultiPacejkaCirclePlanner() : BasePlanner(){};

  /**
   * @brief Example, just returns tracking points of a circle for each car
   */
  std::vector<multi_car_cartesian_reference_point> getPlannedTrajectory(
      const std::map<std::string, crs_models::pacejka_model::pacejka_car_state> state, double timestamp = 0);

  bool goalReached(const std::map<std::string, crs_models::pacejka_model::pacejka_car_state>,
                   const std::vector<multi_car_cartesian_reference_point>& trajectory) override;

  /**
  * @brief returns edges of voronoi partitions as vectors
  */
  std::vector<std::vector<double>> getVorEdgesX();
  std::vector<std::vector<double>> getVorEdgesY();
};
}  // namespace crs_planning
#endif /* SRC_CRS_PLANNING_CIRCLE_PLANNER_INCLUDE_CIRCLE_PLANNER_MULTI_PACEJKA_CIRCLE_PLANNER */
