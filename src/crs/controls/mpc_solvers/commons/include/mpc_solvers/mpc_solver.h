#ifndef MPC_SOLVERS_MPC_SOLVER_H
#define MPC_SOLVERS_MPC_SOLVER_H

namespace mpc_solvers
{
template <typename dynamicsType, typename costType, typename otherParams>
class MpcSolver
{
public:
  /**
   * @brief Get the Dimension of the state
   *
   * @return const int
   */
  virtual const int getStateDimension() const = 0;

  /**
   * @brief Get the Dimension of the input
   *
   * @return const int
   */
  virtual const int getInputDimension() const = 0;

  /**
   * @brief Get the Horizon Length
   *
   * @return const int
   */
  virtual const int getHorizonLength() const = 0;

  /**
   * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
   *
   * @param constraint
   */
  virtual void setInitialState(double constraint[]) = 0;
  /**
   * @brief Sets an initial guess for the state at stage "stage" of the solver.
   * The provided array must have the same length as the state dimension
   *
   * @param constraint
   */
  virtual void setStateInitialGuess(int stage, double constraint[]) = 0;
  /**
   * @brief Sets an initial guess for the input at stage "stage" of the solver.
   * The provided array must have the same length as the input dimension
   *
   * @param constraint
   */
  virtual void setInputInitialGuess(int stage, double constraint[]) = 0;

  /**
   * @brief Updates the internal params for stage "stage"
   *
   * @param stage which stage to update the parameter [0,HorizonLength)
   * @param model_dynamics  the model dynamics
   * @param costs  the cost parameters
   * @param others other parameters (e.g. track point...)
   */
  virtual void updateParams(int stage, const dynamicsType& model_dynamics, const costType& costs,
                            const otherParams& others) = 0;

  /**
  virtual void updateParams(int stage, const crs_models::pacejka_model::pacejka_params& model_dynamics,
                            const tracking_costs& costs, const trajectory_track_point& tracking_point) = 0;
   * @brief Solves the optimization problems and stores the solution in x and u.
   *
   * @param x State array or point with size N*StateDimenstion
   * @param u Input array or point with size N*Inputdimension
   * @return const int, return code. If no error occurred, return code is zero
   */
  virtual int solve(double x[], double u[]) = 0;

  /**
   * @brief Returns the sample period in seconds
   *
   * @return double
   */
  virtual double getSamplePeriod() = 0;
};
}  // namespace mpc_solvers
#endif  // MPC_SOLVERS_MPC_SOLVER_H
