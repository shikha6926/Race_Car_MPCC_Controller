#ifndef MPC_SOLVERS_PACEJKA_TRACKING_MPC_SOLVER_H
#define MPC_SOLVERS_PACEJKA_TRACKING_MPC_SOLVER_H

#include "mpc_solver.h"

#include <pacejka_model/pacejka_params.h>

namespace mpc_solvers
{
namespace pacejka_tracking_solvers
{

/**
 * @brief Enum to easily reference a specific entry of the state array
 *
 */
enum vars
{
  X,
  Y,
  YAW,
  VX,
  VY,
  dYAW,
  TORQUE,
  STEER
};

/**
 * @brief Enum to easily reference a specific entry of the parameter array
 *
 */
enum params
{
  X_LIN,
  Y_LIN,
  Q1,
  Q2,
  R1,
  R2,
  L_REAR,
  L_FRONT,
  m,
  I,
  Df,
  Cf,
  Bf,
  Dr,
  Cr,
  Br,
  Cm1,
  Cm2,
  Cd,
  Croll
};

/**
 * @brief Enum to easily reference a specific entry of the input array
 *
 */
enum inputs
{
  dTORQUE,
  dSTEER
};

/**
 * @brief Struct containing the information for a reference point on the track
 *
 */
struct trajectory_track_point
{
  /**
   * @brief Reference point x coordinate
   *
   */
  double x;
  /**
   * @brief Reference point y coordinate
   *
   */
  double y;
};

/**
 * @brief Struct containing the costs of the mpc problem
 *
 */
struct tracking_costs
{
  /**
   * @brief Contouring Cost
   *
   */
  double Q1;
  /**
   * @brief Lag cost
   *
   */
  double Q2;
  /**
   * @brief dtorque cost
   *
   */
  double R1;
  /**
   * @brief dsteer cost
   *
   */
  double R2;
};

class PacejkaTrackingMpcSolver
  : public MpcSolver<crs_models::pacejka_model::pacejka_params, tracking_costs, trajectory_track_point>
{
public:
  /**
   * @brief Get the Dimension of the state
   *
   * @return const int
   */
  const int getStateDimension() const override
  {
    return 8;
  }

  /**
   * @brief Get the Dimension of the input
   *
   * @return const int
   */
  const int getInputDimension() const override
  {
    return 2;
  }

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
   * @param tracking_point tracking point
   */
  virtual void updateParams(int stage, const crs_models::pacejka_model::pacejka_params& model_dynamics,
                            const tracking_costs& costs, const trajectory_track_point& tracking_point) = 0;

  /**
   * @brief Solves the optimization problems and stores the solution in x and u.
   *
   * @param x State array or point with size N*StateDimenstion
   * @param u Input array or point with size N*Inputdimension
   * @return const int, return code. If no error occurred, return code is zero
   */
  virtual int solve(double x[], double u[]) = 0;
};

}  // namespace pacejka_solvers
}  // namespace mpc_solvers

#endif /* MPC_SOLVERS_PACEJKA_MPCC_SOLVER_H */
