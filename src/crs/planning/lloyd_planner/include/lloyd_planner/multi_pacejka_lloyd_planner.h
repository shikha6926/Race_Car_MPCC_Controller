#ifndef SRC_CRS_PLANNING_LLOYD_PLANNER_INCLUDE_LLOYD_PLANNER_MULTI_PACEJKA_LLOYD_PLANNER
#define SRC_CRS_PLANNING_LLOYD_PLANNER_INCLUDE_LLOYD_PLANNER_MULTI_PACEJKA_LLOYD_PLANNER

extern "C" {
#include "jc_voronoi.h"
}

#include <planning/base_planner.h>
#include <planning/multi_car_cartesian_reference_point.h>
#include <planning/cartesian_reference_point.h>
#include <memory>
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <map>
#include <numeric>

namespace crs_planning
{
class MultiPacejkaLloydPlanner : public BasePlanner<multi_car_cartesian_reference_point,
                                                    std::map<std::string, crs_models::pacejka_model::pacejka_car_state>>
{
private:
  double angle_ = 0.0;
  std::map<std::string, bool> car_to_reach_state_;

  // amount of agents
  int num_agents_;

  // voronoi partition
  std::vector<double> boundary_;
  jcv_rect bounding_box_;

  std::vector<std::vector<std::pair<cartesian_reference_point, cartesian_reference_point>>> vor_cell_edges_;
  std::vector<std::vector<cartesian_reference_point>> vor_cell_vertices_;

  // voronoi centroids - taken as references
  std::vector<cartesian_reference_point> centroids_;

  // storage of last reference
  std::vector<cartesian_reference_point> previous_reference_;

  // storage of initial position
  std::vector<cartesian_reference_point> initial_state_;

  // initialization flags
  bool initial_state_flag_;
  bool previous_reference_flag_;

public:
  MultiPacejkaLloydPlanner() : BasePlanner()
  {
    initial_state_flag_ = true;
    previous_reference_flag_ = false;
  };

  /**
   * creates a vector between start and stop with step size step
   */
  template <typename T>
  std::vector<T> arange(T start, T stop, T step);

  /**
   * @brief returns centroids of current voronoi tessellation
   */
  std::vector<multi_car_cartesian_reference_point> getPlannedTrajectory(
      const std::map<std::string, crs_models::pacejka_model::pacejka_car_state> state, double timestamp = 0);

  /**
   * @brief rset flags of reached goals
   */
  bool goalReached(const std::map<std::string, crs_models::pacejka_model::pacejka_car_state>,
                   const std::vector<multi_car_cartesian_reference_point>& trajectory) override;

  /**
   * @brief returns edges of voronoi partitions as vectors
   */
  std::vector<std::vector<double>> getVorEdgesX();
  std::vector<std::vector<double>> getVorEdgesY();

  /**
   * @brief creates grid for all required integrations
   */
  std::vector<cartesian_reference_point> createGrid(std::vector<cartesian_reference_point>&, int);

  /**
   * @brief checks if point is part of polygon
   */
  bool pnpoly(std::vector<cartesian_reference_point>& vertices, cartesian_reference_point p);

  /**
   * @brief computes voronoi partition
   */
  void computeVoronoiDiagram(const std::map<std::string, crs_models::pacejka_model::pacejka_car_state> state);

  /**
   * @brief calculates density value at point p
   */
  static double sensory_function(cartesian_reference_point p);

  /**
   * @brief computeCentroids of Voronoi partition
   */
  std::vector<cartesian_reference_point> computeCentroids();

  /**
   * @brief returns center of mass
   */
  cartesian_reference_point pointCenterOfMass(std::vector<cartesian_reference_point>& points,
                                              std::vector<double>& values, double mass);

  /**
   * @brief store initial position
   */
  std::vector<cartesian_reference_point>
  setInitialPosition(int num_agents, const std::map<std::string, crs_models::pacejka_model::pacejka_car_state> state);

  /**
   * @brief initialize vector of previous centroids with zero
   */
  std::vector<cartesian_reference_point> initializePreviousCentroids(int num_agents);

  /**
   * @brief check whether centroid has changed
   */
  void check_change_in_centroid(double precision);
};
}  // namespace crs_planning
#endif /* SRC_CRS_PLANNING_CIRCLE_PLANNER_INCLUDE_CIRCLE_PLANNER_MULTI_PACEJKA_CIRCLE_PLANNER */
