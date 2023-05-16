#include "ros_crs_utils/state_message_conversion.h"
#include <cmath>
#include <crs_msgs/car_input.h>
#include <crs_msgs/car_state_cart.h>

#include <kinematic_model/kinematic_params.h>
#include <kinematic_model/kinematic_car_input.h>
#include <kinematic_model/kinematic_car_state.h>

namespace message_conversion
{

template <>
crs_msgs::car_input convertToRosInput(const crs_models::kinematic_model::kinematic_car_input input)
{
  crs_msgs::car_input ros_input;
  ros_input.steer = input.steer;
  ros_input.torque = input.torque;
  ros_input.header.stamp = ros::Time::now();
  return ros_input;
}

template <>
crs_models::kinematic_model::kinematic_car_input convertToCrsInput(const crs_msgs::car_input input)
{
  return crs_models::kinematic_model::kinematic_car_input(input.torque, input.steer);
}

template <>
crs_models::kinematic_model::kinematic_car_state convertMsgToState(const crs_msgs::car_state_cart& msg)
{
  crs_models::kinematic_model::kinematic_car_state state;
  state.pos_x = msg.x;
  state.pos_y = msg.y;
  state.yaw = msg.yaw;
  state.velocity = std::sqrt(std::pow(msg.vx_b, 2) + std::pow(msg.vy_b, 2));
  return state;
}

template <>
crs_msgs::car_state_cart convertStateToRosMsg(const crs_models::kinematic_model::kinematic_car_state& state,
                                              const kinematic_information& info)
{
  crs_msgs::car_state_cart state_msg;
  // Position and orientation
  state_msg.x = state.pos_x;
  state_msg.y = state.pos_y;
  state_msg.yaw = state.yaw;

  auto beta = std::atan2(std::tan(info.input.steer) * info.params.lr, info.params.lr + info.params.lf);
  state_msg.dyaw = state.velocity * std::sin(beta) * 1 / info.params.lr;
  state_msg.vx_b = state.velocity * std::cos(beta);
  state_msg.vy_b = state.velocity * std::sin(beta);

  // Conversion to different coordinates
  state_msg.v_tot = state.velocity;
  state_msg.vx_w = state.velocity * std::cos(state.yaw + beta);
  state_msg.vy_w = state.velocity * std::sin(state.yaw + beta);

  // WARN, not initialized:
  // state_msgs.droll;
  // state_msgs.dpitch;
  // state_msgs.roll;
  // state_msgs.pitch;
  // state_msg.z;
  // state_msg.ax_w;
  // state_msg.ay_w;
  // state_msg.az_w;
  // state_msg.ax_b;
  // state_msg.ay_b;
  // state_msg.az_b;
  // state_msg.steer;

  state_msg.header.stamp = ros::Time::now();
  return state_msg;
}
}  // namespace message_conversion