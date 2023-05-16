#ifndef ROS_CONTROLLERS_COMPONENT_REGISTRY_H
#define ROS_CONTROLLERS_COMPONENT_REGISTRY_H
#include "ros_controllers/ros_controller.h"
#include "ros/ros.h"
namespace ros_controllers
{

/**
 * @brief Returns the ros_controller for the requested types.
 *
 * @tparam StateMsg State Message Type (ros)
 * @tparam InputMsg  Input Message Type (ros)
 * @tparam StateType  State Message Type (crs)
 * @tparam InputType  Input Message Type (crs)
 * @param nh  public nodehandle
 * @param nh_private  private nodehandle
 * @param controller_type type of the controller (e.g. pid, mpcc,... )
 * @param dynamic_callback_allocator  pointer to memory where the reference for the dynamic reconfigure callback is
 * stored, if dynamic configure is defined
 * @return RosController<StateMsg, InputMsg, StateType, InputType>*
 */
template <typename StateMsg, typename InputMsg, typename StateType, typename InputType>
RosController<StateMsg, InputMsg, StateType, InputType>* resolveController(ros::NodeHandle& nh,
                                                                           ros::NodeHandle& nh_private,
                                                                           const std::string& controller_type,
                                                                           void*& dynamic_callback_allocator);

}  // namespace ros_controllers
#endif /* ROS_CONTROLLERS_COMPONENT_REGISTRY_H */
