#ifndef SAFETY_FRAMEWORK_MODEL_BASED_SAFETY_FILTER
#define SAFETY_FRAMEWORK_MODEL_BASED_SAFETY_FILTER
#include <memory>
#include "safety_filter.h"

namespace crs_safety
{
template <typename StateType, typename InputType, typename Model>
class ModelBasedSafetyFilter : public SafetyFilter<StateType, InputType>
{
protected:
  std::shared_ptr<Model> model;

public:
  ModelBasedSafetyFilter(std::shared_ptr<Model> model) : model(model){};
  virtual InputType getSafeControlInput(const StateType state, const InputType control_input) = 0;
};
}  // namespace crs_safety
#endif /* SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_DISCRETE_DYNAMIC_MODEL */
