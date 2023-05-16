
#include "collision_avoider/simple_pacejka_collision_avoider.h"
namespace crs_safety
{

SimplePacejkaCollisionAvoider::SimplePacejkaCollisionAvoider(
    std::shared_ptr<crs_controls::Trajectory> trajectory,
    std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model)
  : ModelBasedSafetyFilter(model), trajectory_(trajectory)
{
}

crs_models::pacejka_model::pacejka_car_input
SimplePacejkaCollisionAvoider::getSafeControlInput(const crs_models::pacejka_model::pacejka_car_state state,
                                                   const crs_models::pacejka_model::pacejka_car_input control_input)
{
  double lookahead_time = 0.2;  // s
  double max_distance = 0.2;    // m
  auto future_state = model->applyModel(state, control_input, lookahead_time);

  Eigen::Vector2d car_pt = Eigen::Vector2d(state.pos_x, state.pos_y);

  int track_idx = trajectory_->getClosestTrackPointIdx(car_pt);
  const Eigen::Vector2d& closest_pt = trajectory_->operator[](track_idx);
  const Eigen::Vector2d& next_pt = trajectory_->operator[](track_idx + 1);  // Operator Handles roll over

  // Calculate Side
  // -1 if car on the left of the track, +1 if car on the right of track
  int side = (((next_pt.x() - closest_pt.x()) * (car_pt.y() - closest_pt.y()) -
               (next_pt.y() - closest_pt.y()) * (car_pt.x() - closest_pt.x())) > 0) ?
                 -1 :
                 1;

  if ((car_pt - closest_pt).norm() >= max_distance)
  {
    std::cout << "detected issue in safety filter. Overwriting!" << std::endl;
    // Only overwrite steer:
    crs_models::pacejka_model::pacejka_car_input u = control_input;
    u.steer = side * 0.4;
    return u;
  }

  return control_input;
};
};  // namespace crs_safety