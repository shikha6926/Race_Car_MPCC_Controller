#ifndef SRC_ROS_ROS_CONTROLLERS_INCLUDE_ROS_CONTROLLERS_DYNAMIC_CONFIG
#define SRC_ROS_ROS_CONTROLLERS_INCLUDE_ROS_CONTROLLERS_DYNAMIC_CONFIG

#ifdef pid_controller_FOUND
#include "ros_controllers/PIDConfig.h"
#include <pid_controller/pacejka_pid_controller.h>
#endif

#ifdef ff_fb_controller_FOUND
#include "ros_controllers/ff_fbConfig.h"
#include <ff_fb_controller/ff_fb_controller.h>
#endif

#ifdef mpc_controller_FOUND
#include "ros_controllers/pacejka_mpccConfig.h"
#include <mpc_controller/pacejka_controller/mpcc_pacejka_controller.h>
#endif

#include <dynamic_reconfigure/server.h>

namespace ros_controllers
{
#ifdef pid_controller_FOUND
/**
 * @brief Class that creates a dynamic_callback parameter server and connects it with the underlying controller
 * Allows to tune a controller using a GUI interface
 *
 */
class DynamicPIDConfigServer
{
protected:
  std::shared_ptr<crs_controls::PacejkaPIDController> controller_;
  dynamic_reconfigure::Server<PIDConfig> server;
  dynamic_reconfigure::Server<PIDConfig>::CallbackType f;
  bool ignored_first_call = false;

public:
  DynamicPIDConfigServer(std::shared_ptr<crs_controls::PacejkaPIDController> controller);
  /**
   * @brief Callback that gets called from the dynamic reconfigure service
   *
   * @param config
   * @param level
   */
  void callback(PIDConfig& config, uint32_t level);
};
#endif

#ifdef ff_fb_controller_FOUND
/**
 * @brief Class that creates a dynamic_callback parameter server and connects it with the underlying controller
 * Allows to tune a controller using a GUI interface
 *
 */
class DynamicFfFbConfigServer
{
protected:
  std::shared_ptr<crs_controls::FfFbController> controller_;
  dynamic_reconfigure::Server<ff_fbConfig> server;
  dynamic_reconfigure::Server<ff_fbConfig>::CallbackType f;
  bool ignored_first_call = false;

public:
  DynamicFfFbConfigServer(std::shared_ptr<crs_controls::FfFbController> controller);
  /**
   * @brief Callback that gets called from the dynamic reconfigure service
   *
   * @param config
   * @param level
   */
  void callback(ff_fbConfig& config, uint32_t level);
};
#endif

#ifdef mpc_controller_FOUND
/**
 * @brief Class that creates a dynamic_callback parameter server and connects it with the underlying controller
 * Allows to tune a controller using a GUI interface
 *
 */
class DynamicPacejkaMPCCConfigServer
{
protected:
  std::shared_ptr<crs_controls::PacejkaMpccController> controller_;
  dynamic_reconfigure::Server<pacejka_mpccConfig> server;
  dynamic_reconfigure::Server<pacejka_mpccConfig>::CallbackType f;
  bool ignored_first_call = false;

public:
  DynamicPacejkaMPCCConfigServer(std::shared_ptr<crs_controls::PacejkaMpccController> controller);
  /**
   * @brief Callback that gets called from the dynamic reconfigure service
   *
   * @param config
   * @param level
   */
  void callback(pacejka_mpccConfig& config, uint32_t level);
};
#endif

}  // namespace ros_controllers
#endif /* SRC_ROS_ROS_CONTROLLERS_INCLUDE_ROS_CONTROLLERS_DYNAMIC_CONFIG */
