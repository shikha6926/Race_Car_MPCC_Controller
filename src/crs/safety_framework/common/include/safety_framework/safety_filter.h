#ifndef SAFETY_FRAMEWORK_SAFETY_FILTER
#define SAFETY_FRAMEWORK_SAFETY_FILTER

namespace crs_safety
{
template <typename StateType, typename InputType>
class SafetyFilter
{
public:
  SafetyFilter(){};

  virtual InputType getSafeControlInput(const StateType state, const InputType control_input) = 0;
};
}  // namespace crs_safety
#endif /* SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_DISCRETE_DYNAMIC_MODEL */
