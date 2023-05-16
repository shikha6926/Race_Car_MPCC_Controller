#include "circle_planner/pacejka_circle_planner.h"

namespace crs_planning
{

std::vector<cartesian_reference_point>
PacejkaCirclePlanner::getPlannedTrajectory(const crs_models::pacejka_model::pacejka_car_state state, double timestamp)
{
  float radius = 2;  // 1m
  float center_x = 0.1;
  float center_y = 0.0;
  std::vector<cartesian_reference_point> pts;
  cartesian_reference_point ref_pt;
  angle_ += 0.1;
  if (angle_ > 160 * M_PI / 180.0)
  {
    angle_ = 0;
    center_y += 0.3;
  }
  ref_pt.x = std::sin(angle_) * radius + center_x;  // + state.pos_x;
  ref_pt.y = std::cos(angle_) * radius + center_y;  // + state.pos_y;
  pts.push_back(ref_pt);
  return pts;
};

bool PacejkaCirclePlanner::goalReached(const crs_models::pacejka_model::pacejka_car_state state,
                                       const std::vector<cartesian_reference_point>& trajectory)
{
  if (trajectory.empty())
  {
    return true;
  }

  auto last_pt = trajectory.at(trajectory.size() - 1);
  // std::cout << "checking goal reached!" << state.pos_x << ", " << state.pos_y << "   :  " << last_pt.x << ", "
  //           << last_pt.y << std::endl;
  // std::cout << "Returning" << Eigen::Vector2d(state.pos_x - last_pt.x, state.pos_y - last_pt.y).norm() << std::endl;
  return Eigen::Vector2d(state.pos_x - last_pt.x, state.pos_y - last_pt.y).norm() < 0.2;
}

std::vector<std::vector<double>> PacejkaCirclePlanner::getVorEdgesX()
{
  std::vector<std::vector<double>> vor_edges_x{{0.0}};
  return vor_edges_x;
}

std::vector<std::vector<double>> PacejkaCirclePlanner::getVorEdgesY()
{
  std::vector<std::vector<double>> vor_edges_y{{0.0}};
  return vor_edges_y;
}

}  // namespace crs_planning
