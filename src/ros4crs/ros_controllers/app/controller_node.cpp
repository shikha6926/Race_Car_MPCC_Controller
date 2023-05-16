#include "ros_controllers/component_registry.h"
#include <ros/ros.h>

#include "ros_controllers/trajectory/dynamic_trajectory_updater.h"
#include "ros_controllers/ros_joystick_controller.h"

#ifdef pacejka_model_FOUND
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#endif

#include <crs_msgs/car_input.h>
#include <crs_msgs/car_state_cart.h>

// Required reference so object does not go out of scope and gets dereferenced
void* controller_ptr;
// Same as before
void* dynamic_callback;

// Only used when dynamically updating trajectories
ros_controllers::DynamicTrajectoryUpdator* trajectory_updator;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_controller");
  ros::NodeHandle nh = ros::NodeHandle("");           // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/ros_controller/*

  std::string state_type;
  std::string input_type;
  std::string controller_type;

  nh_private.getParam("state_type", state_type);
  nh_private.getParam("input_type", input_type);
  nh_private.getParam("controller_type", controller_type);

#ifdef pacejka_model_FOUND
  if (state_type == "pacejka_car" && input_type == "pacejka_car")
  {
    // Check if we want to load the joystick controller
    // It does not fit into the RosController framework thus it is checked here
    if (controller_type == "joystick")
    {
      controller_ptr = ((void*)new ros_controllers::JoystickController(nh, nh_private));
    }
    else
    {
      auto* controller = ros_controllers::resolveController<crs_msgs::car_state_cart, crs_msgs::car_input,
                                                            crs_models::pacejka_model::pacejka_car_state,
                                                            crs_models::pacejka_model::pacejka_car_input>(
          nh, nh_private, controller_type, dynamic_callback);
      // Load controller
      controller_ptr = ((void*)controller);

      // This is a little bit ugly.
      // This makes sure that the trajectory values of the controller
      // can be updated with ros messages
      if (controller_type == "CONST_REF_PID" || controller_type == "TRACKING_MPC")
      {
        trajectory_updator = new ros_controllers::DynamicTrajectoryUpdator(
            nh, nh_private, controller->controller_->getTrajectory<crs_controls::DynamicPointTrajectory>());
      }
    }

    if (!controller_ptr)
    {
      ROS_ERROR_STREAM("Did not find controller for type: " << controller_type << " aborting!");
      return 1;
    }
  }
#endif
  if (!controller_ptr)
  {
    ROS_ERROR_STREAM("Unknown state and input type: " << state_type << " " << input_type << ". Aborting!");
    return 1;
  }
  ros::MultiThreadedSpinner spinner(1);  // Use 1 threads
  spinner.spin();                        // spin() will not return until the node has been shutdown
  return 0;
}
