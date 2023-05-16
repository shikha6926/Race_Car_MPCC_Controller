#ifndef SRC_CRS_CONTROLS_MPC_SOLVERS_COMMONS_INCLUDE_MPC_SOLVERS_PACEJKA_SAFETY_SOLVER
#define SRC_CRS_CONTROLS_MPC_SOLVERS_COMMONS_INCLUDE_MPC_SOLVERS_PACEJKA_SAFETY_SOLVER

#include "mpc_solver.h"

#include <pacejka_model/pacejka_params.h>

namespace mpc_solvers
{
namespace pacejka_safety_solvers
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

enum parameters
{
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
  Croll,

  Car_width,     // Width of the car
  Car_overhang,  // Distance between front wheel axis to front of the car

  xp_track,   // Current reference point on track
  yp_track,   // Current reference point on track
  yaw_track,  // Current reference yaw on track

  xp_e,  // Terminal point
  yp_e,  // Terminal point
  yaw_e  // Terminal point
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

struct reference_input
{
  double torque;
  double steer;

  double cost_torque;
  double cost_steer;

  double cost_delta_torque;
  double cost_delta_steer;
};

/**
 * @brief Struct containing the information for a reference point on the track
 *
 */
struct reference_on_track
{
  double xp_track;
  double yp_track;
  double yaw_track;
  double xp_e;
  double yp_e;
  double yaw_e;
};

class PacejkaSafetySolver
  : public MpcSolver<crs_models::pacejka_model::pacejka_params, reference_input, reference_on_track>
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
                            const reference_input& ref_input, const reference_on_track& reference) = 0;

  /**
   * @brief Solves the optimization problems and stores the solution in x and u.
   *
   * @param x State array or point with size N*StateDimenstion
   * @param u Input array or point with size N*Inputdimension
   * @return const int, return code. If no error occurred, return code is zero
   */
  virtual int solve(double x[], double u[]) = 0;

  /** TODO
   */
  virtual double getSolveTime() = 0;
};

}  // namespace pacejka_safety_solvers
}  // namespace mpc_solvers

#endif /*  */
