#ifndef MPC_CONTROLLER_PACEJKA_CONTROLLER_MPCC_PACEJKA_CONFIG_H
#define MPC_CONTROLLER_PACEJKA_CONTROLLER_MPCC_PACEJKA_CONFIG_H

#include <string.h>

namespace crs_controls
{
struct mpcc_pacejka_config
{
  /**
   * @brief contouring cost
   *
   */
  double Q1;
  /**
   * @brief lag cost
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
  /**
   * @brief  darclength cost
   *
   */
  double R3;
  /**
   * @brief arclength cost
   *
   */
  double q;
  /**
   * @brief State will be propagated by this amount before calculating control input
   *
   */
  double lag_compensation_time;
  /**
   * @brief Type of solver that should be used. e.g. ACADOS
   *
   */
  std::string solver_type;
};
}  // namespace crs_controls

#endif  // MPC_CONTROLLER_PACEJKA_CONTROLLER_MPCC_PACEJKA_CONFIG_H
