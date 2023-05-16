#ifndef SRC_CRS_SAFETY_FRAMEWORK_COLLISION_AVOIDER_INCLUDE_COLLISION_AVOIDER_SIMPLE_PACEJKA_COLLISION_AVOIDER
#define SRC_CRS_SAFETY_FRAMEWORK_COLLISION_AVOIDER_INCLUDE_COLLISION_AVOIDER_SIMPLE_PACEJKA_COLLISION_AVOIDER

#include <safety_framework/model_based_safety_filter.h>

#include <pacejka_model/pacejka_discrete.h>
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_params.h>

#include <commons/trajectory.h>

namespace crs_safety
{

class SimplePacejkaCollisionAvoider : public ModelBasedSafetyFilter<crs_models::pacejka_model::pacejka_car_state,
                                                                    crs_models::pacejka_model::pacejka_car_input,
                                                                    crs_models::pacejka_model::DiscretePacejkaModel>
{
public:
  SimplePacejkaCollisionAvoider(std::shared_ptr<crs_controls::Trajectory> trajectory,
                                std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model);

  crs_models::pacejka_model::pacejka_car_input
  getSafeControlInput(const crs_models::pacejka_model::pacejka_car_state state,
                      const crs_models::pacejka_model::pacejka_car_input control_input) override;

private:
  std::shared_ptr<crs_controls::Trajectory> trajectory_;
};

}  // namespace crs_safety
#endif /* SRC_CRS_SAFETY_FRAMEWORK_COLLISION_AVOIDER_INCLUDE_COLLISION_AVOIDER_SIMPLE_PACEJKA_COLLISION_AVOIDER */
