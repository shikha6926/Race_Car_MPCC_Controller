#include "circle_planner/multi_pacejka_circle_planner.h"
#include <map>
#include <string>
namespace crs_planning
{

std::vector<multi_car_cartesian_reference_point> MultiPacejkaCirclePlanner::getPlannedTrajectory(
    const std::map<std::string, crs_models::pacejka_model::pacejka_car_state> state, double timestamp)
{
  float radius = 1;  // 1m

  std::map<std::string, float> centers_x = { { "car1", 1.0 }, { "car2", 0.5 } };
  std::map<std::string, float> centers_y = { { "car1", 0.0 }, { "car2", 0.5 } };

  std::vector<multi_car_cartesian_reference_point> pts;

  for (int i = 0; i < 1; i++)  // One point for each
  {
    angle_ += 0.1;
    multi_car_cartesian_reference_point reference_pts;
    for (const auto& ns_and_state : state)
    {
      cartesian_reference_point ref_pt;
      ref_pt.x = std::sin(angle_) * radius + centers_x[ns_and_state.first];  // + ns_and_state.second.pos_x;
      ref_pt.y = std::cos(angle_) * radius + centers_y[ns_and_state.first];  // + ns_and_state.second.pos_y;
      reference_pts.points.push_back(ref_pt);
      reference_pts.namespaces.push_back(ns_and_state.first);
    }
    pts.push_back(reference_pts);
  }

  // Reset reached information
  car_to_reach_state_.clear();

  for (const auto car_name_to_state : state)
  {
    car_to_reach_state_[car_name_to_state.first] = false;  // Set goal reached to  false
  }

  return pts;
};

bool MultiPacejkaCirclePlanner::goalReached(
    const std::map<std::string, crs_models::pacejka_model::pacejka_car_state> states,
    const std::vector<multi_car_cartesian_reference_point>& trajectory)
{
  bool found_not_reached = false;

  for (const auto& entry : states)
  {
    const auto& car_reached = car_to_reach_state_.find(entry.first);

    if (car_reached == car_to_reach_state_.end())
    {
      car_to_reach_state_[entry.first] = true;
    }
    else if (!car_reached->second)
    {
      auto last_entry = trajectory.at(trajectory.size() - 1);
      int car_idx = -1;
      for (int i = 0; i < last_entry.namespaces.size(); i++)
      {
        if (last_entry.namespaces[i] == car_reached->first)
        {
          car_idx = i;
          // Found car. Check target reached.
          break;
        }
      }

      if (car_idx == -1)
      {
        std::cout << "ERROR DID NOT FIND CAR FOR NAMESPACE" << car_reached->first << std::endl;
        continue;
      }
      else
      {
        auto last_pt = last_entry.points[car_idx];
        auto state = entry.second;

        car_reached->second = Eigen::Vector2d(state.pos_x - last_pt.x, state.pos_y - last_pt.y).norm() < 0.3;  // 30cm
        if (!car_reached->second)  // did not reach goal
          found_not_reached = true;
      }
    }
  }

  // Debugging
  for (const auto& entry : car_to_reach_state_)
  {
    std::cout << "[" << entry.first << "] - Reached Goal: " << entry.second << std::endl;
  }
  std::cout << "-----" << std::endl;

  return !found_not_reached;
}

std::vector<std::vector<double>> MultiPacejkaCirclePlanner::getVorEdgesX()
{
  std::vector<std::vector<double>> vor_edges_x{{0.0}};
  return vor_edges_x;
}

std::vector<std::vector<double>> MultiPacejkaCirclePlanner::getVorEdgesY()
{
  std::vector<std::vector<double>> vor_edges_y{{0.0}};
  return vor_edges_y;
}

}  // namespace crs_planning
