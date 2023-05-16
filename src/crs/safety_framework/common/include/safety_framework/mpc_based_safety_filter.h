#ifndef SRC_CRS_SAFETY_FRAMEWORK_COMMON_INCLUDE_SAFETY_FRAMEWORK_MPC_BASED_SAFETY_FILTER
#define SRC_CRS_SAFETY_FRAMEWORK_COMMON_INCLUDE_SAFETY_FRAMEWORK_MPC_BASED_SAFETY_FILTER

#include <mpc_solvers/mpc_solver.h>
#include <safety_framework/model_based_safety_filter.h>

namespace crs_safety
{
template <typename StateType, typename InputType, typename Model, typename MpcModel, typename MpcCost,
          typename MpcParams>
class MpcBasedSafetyFilter : public ModelBasedSafetyFilter<StateType, InputType, Model>
{
protected:
  std::shared_ptr<mpc_solvers::MpcSolver<MpcModel, MpcCost, MpcParams>> solver;

  virtual std::shared_ptr<mpc_solvers::MpcSolver<MpcModel, MpcCost, MpcParams>> getSolver(std::string solver_type) = 0;

public:
  MpcBasedSafetyFilter(std::string solver_type, std::shared_ptr<Model> model)
    : ModelBasedSafetyFilter<StateType, InputType, Model>(model){};

  virtual InputType getSafeControlInput(const StateType state, const InputType control_input) = 0;
};
}  // namespace crs_safety
#endif /* SRC_CRS_SAFETY_FRAMEWORK_COMMON_INCLUDE_SAFETY_FRAMEWORK_MPC_BASED_SAFETY_FILTER */
