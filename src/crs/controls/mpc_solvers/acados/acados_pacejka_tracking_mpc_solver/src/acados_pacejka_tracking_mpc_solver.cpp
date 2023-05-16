#include "acados_pacejka_tracking_mpc_solver/acados_pacejka_tracking_mpc_solver.h"

namespace mpc_solvers
{
namespace pacejka_tracking_solvers
{

/**
 * @brief Internal function to set initial guesses for the solver output variable
 *       This function just wraps the ocp_nlp_out_set call
 *
 * @param stage current stage of the mpc (0,....,horizon-1)
 * @param type type either "x" or "u"
 * @param constraint constraints must have same dimension as x or u (depending on type)
 */
void AcadosPacejkaTrackingMpcSolver::setOutputInitialGuess(int stage, std::string type, double constraint[])
{
  ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), stage, type.c_str(), constraint);
}
/**
 * @brief Sets an input bound constraint.
 *  This function just wraps the ocp_nlp_constraints_model_set call
 *
 * @param stage current stage of the mpc (0,....,horizon-1)
 * @param type type either "x" or "u"
 * @param constraint constraints must have same dimension as x or u (depending on type)
 */
void AcadosPacejkaTrackingMpcSolver::setInputBoundConstraint(int stage, std::string type, double constraint[])
{
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, type.c_str(), constraint);
}
/**
 * @brief Get the Last Solution and stores it in x and u
 *
 * @param x states, size state dimension x horizon length
 * @param u input, size input dimension x horizon length
 */
void AcadosPacejkaTrackingMpcSolver::getLastSolution(double x[], double u[])
{
  for (int n = 0; n < getHorizonLength(); n++)
  {
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "x", &x[n * getStateDimension()]);  // NOLINT
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "u", &u[n * getInputDimension()]);  // NOLINT
  }
}

/**
 * @brief Construct a new Acados Pacejka Mpcc Solver:: Acados Pacejka Mpcc Solver object
 *
 */
AcadosPacejkaTrackingMpcSolver::AcadosPacejkaTrackingMpcSolver()
{
  // initialize acados solver
  acados_ocp_capsule_.reset(pacejka_model_tracking_mpc_acados_create_capsule());
  pacejka_model_tracking_mpc_acados_create(acados_ocp_capsule_.get());

  nlp_config_.reset(pacejka_model_tracking_mpc_acados_get_nlp_config(acados_ocp_capsule_.get()));
  nlp_dims_.reset(pacejka_model_tracking_mpc_acados_get_nlp_dims(acados_ocp_capsule_.get()));
  nlp_in_.reset(pacejka_model_tracking_mpc_acados_get_nlp_in(acados_ocp_capsule_.get()));
  nlp_out_.reset(pacejka_model_tracking_mpc_acados_get_nlp_out(acados_ocp_capsule_.get()));
  nlp_solver_.reset(pacejka_model_tracking_mpc_acados_get_nlp_solver(acados_ocp_capsule_.get()));
}

/**
 * @brief Get the Horizon Length
 *
 * @return const int
 */
const int AcadosPacejkaTrackingMpcSolver::getHorizonLength() const
{
  return nlp_dims_->N;
}

/**
 * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
 *
 * @param constraint
 */
void AcadosPacejkaTrackingMpcSolver::setInitialState(double constraint[])
{
  setInputBoundConstraint(0, "lbx", constraint);
  setInputBoundConstraint(0, "ubx", constraint);
}
/**
 * @brief Sets an initial guess for the state at stage "stage" of the solver.
 * The provided array must have the same length as the state dimension
 *
 * @param constraint
 */
void AcadosPacejkaTrackingMpcSolver::setStateInitialGuess(int stage, double constraint[])
{
  setOutputInitialGuess(stage, "x", constraint);
}
/**
 * @brief Sets an initial guess for the input at stage "stage" of the solver.
 * The provided array must have the same length as the input dimension
 *
 * @param constraint
 */
void AcadosPacejkaTrackingMpcSolver::setInputInitialGuess(int stage, double constraint[])
{
  setOutputInitialGuess(stage, "u", constraint);
}
/**
 * @brief Updates the internal params for stage "stage"
 *
 * @param stage which stage to update the parameter [0,HorizonLength)
 * @param model_dynamics  the model dynamics
 * @param costs  the cost parameters
 * @param tracking_point tracking point
 */
void AcadosPacejkaTrackingMpcSolver::updateParams(int stage,
                                                  const crs_models::pacejka_model::pacejka_params& model_dynamics,
                                                  const tracking_costs& costs,
                                                  const trajectory_track_point& tracking_point)
{
  double params[np_];
  // Tracking
  mpc_parameters[params::X_LIN] = tracking_point.x;
  mpc_parameters[params::Y_LIN] = tracking_point.y;

  // Costs
  mpc_parameters[params::Q1] = costs.Q1;
  mpc_parameters[params::Q2] = costs.Q2;
  mpc_parameters[params::R1] = costs.R1;
  mpc_parameters[params::R2] = costs.R2;

  // Dynamics
  mpc_parameters[params::L_REAR] = model_dynamics.lr;
  mpc_parameters[params::L_FRONT] = model_dynamics.lf;
  mpc_parameters[params::m] = model_dynamics.m;
  mpc_parameters[params::I] = model_dynamics.I;
  mpc_parameters[params::Df] = model_dynamics.Df;
  mpc_parameters[params::Cf] = model_dynamics.Cf;
  mpc_parameters[params::Bf] = model_dynamics.Bf;
  mpc_parameters[params::Dr] = model_dynamics.Dr;
  mpc_parameters[params::Cr] = model_dynamics.Cr;
  mpc_parameters[params::Br] = model_dynamics.Br;
  mpc_parameters[params::Cm1] = model_dynamics.Cm1;
  mpc_parameters[params::Cm2] = model_dynamics.Cm2;
  mpc_parameters[params::Cd] = model_dynamics.Cd;
  mpc_parameters[params::Croll] = model_dynamics.Croll;

  pacejka_model_tracking_mpc_acados_update_params(acados_ocp_capsule_.get(), stage, mpc_parameters, np_);
}

/**
 * @brief Solves the optimization problems and stores the solution in x and u.
 *
 * @param x State array or point with size N*StateDimenstion
 * @param u Input array or point with size N*Inputdimension
 * @return const int, return code. If no error occurred, return code is zero
 */
int AcadosPacejkaTrackingMpcSolver::solve(double x[], double u[])
{
  int status = pacejka_model_tracking_mpc_acados_solve(acados_ocp_capsule_.get());
  if (status)
  {
    return status;
  }
  getLastSolution(x, u);
  return status;
}

double AcadosPacejkaTrackingMpcSolver::getSamplePeriod()
{
  return *(nlp_in_->Ts);
}
};  // namespace pacejka_tracking_solvers

}  // namespace mpc_solvers
