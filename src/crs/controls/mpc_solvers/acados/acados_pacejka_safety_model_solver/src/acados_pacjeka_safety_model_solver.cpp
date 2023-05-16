#include "acados_pacejka_safety_model_solver/acados_pacjeka_safety_model_solver.h"

namespace mpc_solvers
{
namespace pacejka_safety_solvers
{

/**
 * @brief Internal function to set initial guesses for the solver output variable
 *       This function just wraps the ocp_nlp_out_set call
 *
 * @param stage current stage of the mpc (0,....,horizon-1)
 * @param type type either "x" or "u"
 * @param constraint constraints must have same dimension as x or u (depending on type)
 */
void AcadosPacejkaSafetySolver::setOutputInitialGuess(int stage, std::string type, double constraint[])
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
void AcadosPacejkaSafetySolver::setInputBoundConstraint(int stage, std::string type, double constraint[])
{
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, type.c_str(), constraint);
}
/**
 * @brief Get the Last Solution and stores it in x and u
 *
 * @param x states, size state dimension x horizon length
 * @param u input, size input dimension x horizon length
 */
void AcadosPacejkaSafetySolver::getLastSolution(double x[], double u[])
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
AcadosPacejkaSafetySolver::AcadosPacejkaSafetySolver()
{
  // initialize acados solver
  acados_ocp_capsule_.reset(safety_dynamic_model_acados_create_capsule());
  safety_dynamic_model_acados_create(acados_ocp_capsule_.get());

  nlp_config_.reset(safety_dynamic_model_acados_get_nlp_config(acados_ocp_capsule_.get()));
  nlp_dims_.reset(safety_dynamic_model_acados_get_nlp_dims(acados_ocp_capsule_.get()));
  nlp_in_.reset(safety_dynamic_model_acados_get_nlp_in(acados_ocp_capsule_.get()));
  nlp_out_.reset(safety_dynamic_model_acados_get_nlp_out(acados_ocp_capsule_.get()));
  nlp_solver_.reset(safety_dynamic_model_acados_get_nlp_solver(acados_ocp_capsule_.get()));
}

/**
 * @brief Get the Horizon Length
 *
 * @return const int
 */
const int AcadosPacejkaSafetySolver::getHorizonLength() const
{
  return nlp_dims_->N;
}

/**
 * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
 *
 * @param constraint
 */
void AcadosPacejkaSafetySolver::setInitialState(double constraint[])
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
void AcadosPacejkaSafetySolver::setStateInitialGuess(int stage, double constraint[])
{
  setOutputInitialGuess(stage, "x", constraint);
}
/**
 * @brief Sets an initial guess for the input at stage "stage" of the solver.
 * The provided array must have the same length as the input dimension
 *
 * @param constraint
 */
void AcadosPacejkaSafetySolver::setInputInitialGuess(int stage, double constraint[])
{
  setOutputInitialGuess(stage, "u", constraint);
}
/**
 * @brief Updates the internal parameters for stage "stage"
 *
 * @param stage which stage to update the parameter [0,HorizonLength)
 * @param model_dynamics  the model dynamics
 * @param ref_input  the cost parameters
 * @param tracking_point tracking point
 */
void AcadosPacejkaSafetySolver::updateParams(int stage, const crs_models::pacejka_model::pacejka_params& model_dynamics,
                                             const reference_input& ref_input, const reference_on_track& safe_set)
{
  // ref_input

  if (stage == 1)
  {
    double W_safety[4 * 4] = { 0.0 };
    W_safety[0 * 4] = ref_input.cost_torque;
    W_safety[1 * 4 + 1] = ref_input.cost_steer;
    W_safety[2 * 4 + 2] = ref_input.cost_delta_torque;
    W_safety[3 * 4 + 3] = ref_input.cost_delta_steer;

    ocp_nlp_cost_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, "W", W_safety);
  }
  else if (stage > 1)
  {
    double W[4 * 4] = { 0.0 };
    W[2 * 4 + 2] = ref_input.cost_delta_torque;
    W[3 * 4 + 3] = ref_input.cost_delta_steer;
    ocp_nlp_cost_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, "W", W);
  }

  double y_ref[4] = { ref_input.torque, ref_input.steer, 0.0, 0.0 };
  ocp_nlp_cost_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), 1, "yref", y_ref);

  // Dynamics
  mpc_params[parameters::L_REAR] = model_dynamics.lr;
  mpc_params[parameters::L_FRONT] = model_dynamics.lf;
  mpc_params[parameters::m] = model_dynamics.m;
  mpc_params[parameters::I] = model_dynamics.I;
  mpc_params[parameters::Df] = model_dynamics.Df;
  mpc_params[parameters::Cf] = model_dynamics.Cf;
  mpc_params[parameters::Bf] = model_dynamics.Bf;
  mpc_params[parameters::Dr] = model_dynamics.Dr;
  mpc_params[parameters::Cr] = model_dynamics.Cr;
  mpc_params[parameters::Br] = model_dynamics.Br;
  mpc_params[parameters::Cm1] = model_dynamics.Cm1;
  mpc_params[parameters::Cm2] = model_dynamics.Cm2;
  mpc_params[parameters::Cd] = model_dynamics.Cd;
  mpc_params[parameters::Croll] = model_dynamics.Croll;

  // TODO: Hardcoded!!!!!!
  mpc_params[parameters::Car_width] = 0.09;
  mpc_params[parameters::Car_overhang] = 0.04;

  // Reference on track
  mpc_params[parameters::xp_track] = safe_set.xp_track;
  mpc_params[parameters::yp_track] = safe_set.yp_track;
  mpc_params[parameters::yaw_track] = safe_set.yaw_track;

  // Final terminal point
  mpc_params[parameters::xp_e] = safe_set.xp_e;
  mpc_params[parameters::yp_e] = safe_set.yp_e;
  mpc_params[parameters::yaw_e] = safe_set.yaw_e;

  safety_dynamic_model_acados_update_params(acados_ocp_capsule_.get(), stage, mpc_params, np_);
}

/**
 * @brief Solves the optimization problems and stores the solution in x and u.
 *
 *
 *  0 – success
 *  1 – failure
 *  2 – maximum number of iterations reached
 *  3 – minimum step size in QP solver reached
 *  4 – qp solver failed
 *
 * @param x State array or point with size N*StateDimenstion
 * @param u Input array or point with size N*Inputdimension
 * @return const int, return code. If no error occurred, return code is zero
 */
int AcadosPacejkaSafetySolver::solve(double x[], double u[])
{
  int status = safety_dynamic_model_acados_solve(acados_ocp_capsule_.get());
  if (status)
  {
    std::cout << "Solver Error. Status: " << status << std::endl;
    return status;
  }
  getLastSolution(x, u);

  return status;
}

double AcadosPacejkaSafetySolver::getSolveTime()
{
  double elapsed_time;
  ocp_nlp_get(nlp_config_.get(), nlp_solver_.get(), "time_tot", &elapsed_time);
  return elapsed_time;
}

double AcadosPacejkaSafetySolver::getSamplePeriod()
{
  return *(nlp_in_->Ts);
}
};  // namespace pacejka_safety_solvers

}  // namespace mpc_solvers