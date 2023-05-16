#include "ros_controllers/component_registry.h"
#include "ros_controllers/ros_controller.h"
#include <ros/ros.h>
#include <ros_crs_utils/parameter_io.h>

#include <commons/dynamic_point_trajectory.h>

#ifdef pacejka_model_FOUND
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#endif

#ifdef pid_controller_FOUND
#include "ros_controllers/PIDConfig.h"
#include <pid_controller/pacejka_pid_controller.h>
#include <pid_controller/pacejka_pid_const_ref_controller.h>
#endif

#ifdef ff_fb_controller_FOUND
#include <ff_fb_controller/ff_fb_controller.h>
#endif

#ifdef mpc_controller_FOUND
#include <mpc_controller/pacejka_controller/mpcc_pacejka_controller.h>
#include <mpc_controller/pacejka_controller/tracking_mpc_pacejka_controller.h>
#endif

#ifdef pacejka_model_FOUND
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_params.h>
#endif

#include "ros_controllers/dynamic_config.h"
#include "ros_controllers/visualizers/last_reference_point_visualizer.h"
#include "ros_controllers/visualizers/mpc_controller_visualizer.h"
#include "ros_controllers/visualizers/lloyd_visualizer.h"
#include "ros_controllers/visualizers/lloyd_plus_mpc_visualizer.h"

#include <crs_msgs/car_state_cart.h>
#include <crs_msgs/car_input.h>

/**
 * @brief This file loads the specific controller implementation and wraps it inside a ros controller object
 *
 */
namespace ros_controllers
{
template <typename ModelType, typename StateType, typename InputType>
std::unique_ptr<BaseControllerVisualizer<StateType, InputType>>
loadControllerVisualizer(ros::NodeHandle nh,
                         std::shared_ptr<crs_controls::BaseController<StateType, InputType>> controller)
{
  std::string type;
  if (!nh.getParam("type", type))
  {
    ROS_WARN("No Type specified for visualizer! No visualizer loaded for controller");
    return std::unique_ptr<BaseControllerVisualizer<StateType, InputType>>(nullptr);
  }

  if (type == "last_reference")
  {
    return std::make_unique<LastReferencePointVisualizer<StateType, InputType>>(nh, controller);
  }

  if (type == "lloyd_pid")
  {
    return std::make_unique<LloydVisualizer<StateType, InputType>>(nh, controller);
  }

  if (type == "mpc")
  {
    return std::make_unique<MpcControllerVisualizer<ModelType, StateType, InputType>>(
        nh, std::dynamic_pointer_cast<crs_controls::MpcController<ModelType, StateType, InputType>>(controller));
  }

  if (type == "lloyd_plus_mpc")
  {
    return std::make_unique<LloydPlusMPCVisualizer<ModelType, StateType, InputType>>(
        nh, std::dynamic_pointer_cast<crs_controls::MpcController<ModelType, StateType, InputType>>(controller));
  }

  ROS_WARN_STREAM("No Visualizer found for type: " << type << ". No visualizer loaded for controller!");
  return std::unique_ptr<BaseControllerVisualizer<StateType, InputType>>(nullptr);
};

//#ifdef pacejka_model_FOUND
// Shorten type names
typedef crs_msgs::car_state_cart ros_car_state;
typedef crs_msgs::car_input ros_car_input;
typedef crs_models::pacejka_model::pacejka_car_state pacejka_state;
typedef crs_models::pacejka_model::pacejka_car_input pacejka_input;
typedef crs_models::pacejka_model::pacejka_params pacejka_model_params;
typedef RosController<ros_car_state, ros_car_input, pacejka_state, pacejka_input> pacejka_ros_controller;

// =============================================================================================
// ===============                  PID CONTROLLER                  ============================
// =============================================================================================
#ifdef pid_controller_FOUND
inline pacejka_ros_controller* getPacejkaPidController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                       void*& dynamic_callback_allocator)
{
  // config from params
  crs_controls::pid_config pid_config =
      parameter_io::getConfig<crs_controls::pid_config>(ros::NodeHandle(nh_private, "controller_params"));

  // Create controller
  auto ptr = std::make_shared<crs_controls::PacejkaPIDController>(
      pid_config, parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track")));

  // Downcast to BaseController type
  auto derived_ptr = std::dynamic_pointer_cast<crs_controls::BaseController<pacejka_state, pacejka_input>>(ptr);
  // Create Visualizer
  auto visualizer_ptr = loadControllerVisualizer<void, pacejka_state, pacejka_input>(
      ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

  dynamic_callback_allocator = (void*)new ros_controllers::DynamicPIDConfigServer(ptr);
  return new pacejka_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}

inline pacejka_ros_controller* getPacejkaConstReferencePidController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                                     void*& dynamic_callback_allocator)
{
  // config from params
  crs_controls::pid_const_ref_config pid_config =
      parameter_io::getConfig<crs_controls::pid_const_ref_config>(ros::NodeHandle(nh_private, "controller_params"));

  // start track points
  std::vector<double> x_start = { 0.3 };
  std::vector<double> y_start = { 0.0 };
  // Create controller
  auto ptr = std::make_shared<crs_controls::PacejkaConstRefPIDController>(
      pid_config, std::static_pointer_cast<crs_controls::Trajectory>(
                      std::make_shared<crs_controls::DynamicPointTrajectory>(x_start, y_start)));

  // Downcast to BaseController type
  auto derived_ptr = std::dynamic_pointer_cast<crs_controls::BaseController<pacejka_state, pacejka_input>>(ptr);
  // Create Visualizer
  auto visualizer_ptr = loadControllerVisualizer<void, pacejka_state, pacejka_input>(
      ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

  return new pacejka_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}

//#endif  // pid_controller_FOUND

// =============================================================================================
// ===============                FF FB CONTROLLER                  ============================
// =============================================================================================

#ifdef ff_fb_controller_FOUND
inline pacejka_ros_controller* getPacejkaFfFbController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                        void*& dynamic_callback_allocator)
{
  // Load FfFbConfig config from params
  crs_controls::FfFbConfig ff_fb_config =
      parameter_io::getConfig<crs_controls::FfFbConfig>(ros::NodeHandle(nh_private, "controller_params"));
  // Load model from params
  std::shared_ptr<pacejka_model_params> pacejka_params = std::make_shared<pacejka_model_params>();
  // First load generic, gt model
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh, "model/model_params"), *pacejka_params);
  // Patch certain params
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh_private, "model/model_params"), *pacejka_params,
                                                     false);
  // Create Controller
  auto ptr = std::make_shared<crs_controls::FfFbController>(
      ff_fb_config, pacejka_params, parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track")));

  // Downcast to BaseController type
  auto derived_ptr = std::dynamic_pointer_cast<crs_controls::BaseController<pacejka_state, pacejka_input>>(ptr);
  // Create Visualizer

  auto visualizer_ptr = loadControllerVisualizer<pacejka_model_params, pacejka_state, pacejka_input>(
      ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

  dynamic_callback_allocator = (void*)new ros_controllers::DynamicFfFbConfigServer(ptr);
  return new pacejka_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}
#endif  // ff_fb_controller_FOUND

// =============================================================================================
// ===============                 MPCC CONTROLLER                  ============================
// =============================================================================================

#ifdef mpc_controller_FOUND
inline pacejka_ros_controller* getPacejkaMPCCController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                        void*& dynamic_callback_allocator)
{
  crs_controls::mpcc_pacejka_config cfg =
      parameter_io::getConfig<crs_controls::mpcc_pacejka_config>(ros::NodeHandle(nh_private, "controller_params"));

  // Load model from params
  pacejka_model_params pacejka_params;

  // First load generic, gt model
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh, "model/model_params"), pacejka_params);

  // Patch certain params
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh_private, "model/model_params"), pacejka_params,
                                                     false);

  // Load model from params
  std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> pacejka_model =
      std::make_shared<crs_models::pacejka_model::DiscretePacejkaModel>(pacejka_params);

  // Create Controller
  auto ptr = std::make_shared<crs_controls::PacejkaMpccController>(
      cfg, pacejka_model, parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track")));

  // Downcast to BaseController type
  auto derived_ptr = std::dynamic_pointer_cast<
      crs_controls::MpcController<crs_models::pacejka_model::DiscretePacejkaModel, pacejka_state, pacejka_input>>(ptr);

  auto visualizer_ptr =
      loadControllerVisualizer<crs_models::pacejka_model::DiscretePacejkaModel, pacejka_state, pacejka_input>(
          ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

  dynamic_callback_allocator = (void*)new ros_controllers::DynamicPacejkaMPCCConfigServer(ptr);
  return new pacejka_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}

inline pacejka_ros_controller* getPacejkaTrackingMPCController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                               void*& dynamic_callback_allocator)
{
  crs_controls::tracking_mpc_pacejka_config cfg = parameter_io::getConfig<crs_controls::tracking_mpc_pacejka_config>(
      ros::NodeHandle(nh_private, "controller_params"));

  // Load model from params
  pacejka_model_params pacejka_params;

  // First load generic, gt model
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh, "model/model_params"), pacejka_params);

  // Patch certain params
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh_private, "model/model_params"), pacejka_params,
                                                     false);

  // Load model from params
  std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> pacejka_model =
      std::make_shared<crs_models::pacejka_model::DiscretePacejkaModel>(pacejka_params);

  // start track points
  std::vector<double> x_start = { 0.3 };
  std::vector<double> y_start = { 0.0 };

  // Create Controller
  auto ptr = std::make_shared<crs_controls::PacejkaTrackingMpcController>(
      cfg, pacejka_model,
      std::static_pointer_cast<crs_controls::Trajectory>(
          std::make_shared<crs_controls::DynamicPointTrajectory>(x_start, y_start)));

  // Downcast to BaseController type
  auto derived_ptr = std::dynamic_pointer_cast<
      crs_controls::MpcController<crs_models::pacejka_model::DiscretePacejkaModel, pacejka_state, pacejka_input>>(ptr);

  auto visualizer_ptr =
      loadControllerVisualizer<crs_models::pacejka_model::DiscretePacejkaModel, pacejka_state, pacejka_input>(
          ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

  // dynamic_callback_allocator = (void*)new ros_controllers::DynamicPacejkaMPCCConfigServer(ptr); //not existant at the
  // moment
  return new pacejka_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}
#endif  // mpc_controller_FOUND

// =============================================================================================
// ===============                 RESOLVE CONTROLLER               ============================
// =============================================================================================
template <>
pacejka_ros_controller* resolveController<ros_car_state, ros_car_input, pacejka_state, pacejka_input>(
    ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& controller_type,
    void*& dynamic_callback_allocator)
{
#ifdef pid_controller_FOUND
  if (controller_type == "PID")
  {
    return getPacejkaPidController(nh, nh_private, dynamic_callback_allocator);
  }

  if (controller_type == "CONST_REF_PID")
  {
    return getPacejkaConstReferencePidController(nh, nh_private, dynamic_callback_allocator);
  }
#endif

#ifdef ff_fb_controller_FOUND
  if (controller_type == "FF_FB")
  {
    return getPacejkaFfFbController(nh, nh_private, dynamic_callback_allocator);
  }
#endif

#ifdef mpc_controller_FOUND
  if (controller_type == "MPCC")
  {
    return getPacejkaMPCCController(nh, nh_private, dynamic_callback_allocator);
  }

  if (controller_type == "TRACKING_MPC")
  {
    return getPacejkaTrackingMPCController(nh, nh_private, dynamic_callback_allocator);
  }
#endif

  assert(true && "Did not find registered controller for specified controller type.");
  return nullptr;
}
#endif

}  // namespace ros_controllers
