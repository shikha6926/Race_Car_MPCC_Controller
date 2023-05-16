#ifndef SRC_CRS_CONTROLS_MPC_SOLVERS_ACADOS_ACADOS_PACJEKA_SAFETY_MODEL_SOLVER_INCLUDE_ACADOS_PACEJKA_MPCC_SOLVER_ACADOS_PACJEKA_SAFETY_MODEL_SOLVER
#define SRC_CRS_CONTROLS_MPC_SOLVERS_ACADOS_ACADOS_PACJEKA_SAFETY_MODEL_SOLVER_INCLUDE_ACADOS_PACEJKA_MPCC_SOLVER_ACADOS_PACJEKA_SAFETY_MODEL_SOLVER

#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"

#include "mpc_solvers/pacejka_safety_solver.h"
#include <pacejka_model/pacejka_params.h>

#include <acados_pacejka_safety_model_solver/acados_solver_safety_dynamic_model.h>

#include <memory>

namespace mpc_solvers
{
namespace pacejka_safety_solvers
{

class AcadosPacejkaSafetySolver : public PacejkaSafetySolver
{
private:
  std::unique_ptr<safety_dynamic_model_solver_capsule> acados_ocp_capsule_;
  std::unique_ptr<ocp_nlp_config> nlp_config_;
  std::unique_ptr<ocp_nlp_dims> nlp_dims_;
  std::unique_ptr<ocp_nlp_in> nlp_in_;
  std::unique_ptr<ocp_nlp_out> nlp_out_;
  std::unique_ptr<ocp_nlp_solver> nlp_solver_;

  /**
   * @brief Numbers of parameters
   *
   */
  const static int np_ = 22;
  int solve_cnt_ = 0;
  double mpc_params[np_];
  double avg_elapsed_total_ = 0.0;
  double avg_elapsed_ = 0.0;

  /**
   * @brief Internal function to set initial guesses for the solver output variable
   *       This function just wraps the ocp_nlp_out_set call
   *
   * @param stage current stage of the mpc (0,....,horizon-1)
   * @param type type either "x" or "u"
   * @param constraint constraints must have same dimension as x or u (depending on type)
   */
  void setOutputInitialGuess(int stage, std::string type, double constraint[]);

  /**
   * @brief Sets an input bound constraint.
   *  This function just wraps the ocp_nlp_constraints_model_set call
   *
   * @param stage current stage of the mpc (0,....,horizon-1)
   * @param type type either "x" or "u"
   * @param constraint constraints must have same dimension as x or u (depending on type)
   */
  void setInputBoundConstraint(int stage, std::string type, double constraint[]);
  /**
   * @brief Get the Last Solution and stores it in x and u
   *
   * @param x states, size state dimension x horizon length
   * @param u input, size state dimension x horizon length
   */
  void getLastSolution(double x[], double u[]);

public:
  AcadosPacejkaSafetySolver();

  /**
   * @brief Get the Horizon Length
   *
   * @return const int
   */
  const int getHorizonLength() const override;

  /**
   * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
   *
   * @param constraint
   */
  void setInitialState(double constraint[]) override;
  /**
   * @brief Sets an initial guess for the state at stage "stage" of the solver.
   * The provided array must have the same length as the state dimension
   *
   * @param constraint
   */
  void setStateInitialGuess(int stage, double constraint[]) override;
  /**
   * @brief Sets an initial guess for the input at stage "stage" of the solver.
   * The provided array must have the same length as the input dimension
   *
   * @param constraint
   */
  void setInputInitialGuess(int stage, double constraint[]) override;
  /**
   * @brief Updates the internal params for stage "stage"
   *
   * @param stage which stage to update the parameter [0,HorizonLength)
   * @param model_dynamics  the model dynamics
   * @param ref_input  the cost parameters
   * @param tracking_point tracking point
   */
  void updateParams(int stage, const crs_models::pacejka_model::pacejka_params& model_dynamics,
                    const reference_input& ref_input, const reference_on_track& tracking_point) override;

  /**
   * @brief Solves the optimization problems and stores the solution in x and u.
   *
   * @param x State array or point with size N*StateDimenstion
   * @param u Input array or point with size N*Inputdimension
   * @return const int, return code. If no error occurred, return code is zero
   */
  int solve(double x[], double u[]) override;

  /**
   * @brief Return the amount of time in seconds it took to solve the optimization problem
   *
   * @return double
   */
  double getSolveTime() override;

  /**
   * @brief Returns the sample period in seconds
   *
   * @return double
   */
  double getSamplePeriod() override;
};

}  // namespace pacejka_safety_solvers
}  // namespace mpc_solvers
#endif /* SRC_CRS_CONTROLS_MPC_SOLVERS_ACADOS_ACADOS_PACJEKA_SAFETY_MODEL_SOLVER_INCLUDE_ACADOS_PACEJKA_MPCC_SOLVER_ACADOS_PACJEKA_SAFETY_MODEL_SOLVER \
        */
