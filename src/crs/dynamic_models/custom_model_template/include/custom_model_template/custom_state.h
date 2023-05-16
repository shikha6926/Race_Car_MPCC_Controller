#ifndef CUSTOM_MODEL_TEMPLATE_CUSTOM_STATE_H
#define CUSTOM_MODEL_TEMPLATE_CUSTOM_STATE_H

#include <iostream>

namespace crs_models
{
namespace custom_model
{

struct custom_state
{
  double pos_x;
  double pos_y;
  double yaw;
  double vel_x;
  double vel_y;
  double yaw_rate;
  custom_state() : pos_x(0), pos_y(0), yaw(0), vel_x(0), vel_y(0), yaw_rate(0){};  // default constructor
  custom_state(double pos_x, double pos_y, double yaw, double vel_x, double vel_y, double yaw_rate)
    : pos_x(pos_x), pos_y(pos_y), yaw(yaw), vel_x(vel_x), vel_y(vel_y), yaw_rate(yaw_rate){};  // Constructor
};

inline custom_state operator+(const custom_state& a, const custom_state& b)
{
  custom_state added_struct;

  added_struct.pos_x = a.pos_x + b.pos_x;
  added_struct.pos_y = a.pos_y + b.pos_y;
  added_struct.yaw = a.yaw + b.yaw;
  added_struct.vel_x = a.vel_x + b.vel_x;
  added_struct.vel_y = a.vel_y + b.vel_y;
  added_struct.yaw_rate = a.yaw_rate + b.yaw_rate;

  return added_struct;
}

inline custom_state operator*(double b, const custom_state& a)
{
  custom_state product_struct;

  product_struct.pos_x = a.pos_x * b;
  product_struct.pos_y = a.pos_y * b;
  product_struct.yaw = a.yaw * b;
  product_struct.vel_x = a.vel_x * b;
  product_struct.vel_y = a.vel_y * b;
  product_struct.yaw_rate = a.yaw_rate * b;

  return product_struct;
}

inline void operator+=(custom_state& a, const custom_state& b)
{
  a.pos_x = a.pos_x + b.pos_x;
  a.pos_y = a.pos_y + b.pos_y;
  a.yaw = a.yaw + b.yaw;
  a.vel_x = a.vel_x + b.vel_x;
  a.vel_y = a.vel_y + b.vel_y;
  a.yaw_rate = a.yaw_rate + b.yaw_rate;
}

inline bool operator==(const custom_state& a, const custom_state& b)
{
  return (a.pos_x == b.pos_x) && (a.pos_y == b.pos_y) && (a.yaw == b.yaw) && (a.vel_x == b.vel_x) &&
         (a.vel_y == b.vel_y) && (a.yaw_rate == b.yaw_rate);
}

inline std::ostream& operator<<(std::ostream& os,
                                const custom_state& state)  // how to print struct (state) to output stream (os)
{
  os << "custom_state:" << std::endl;                                  // Type of struct (state)
  os << " Pos_x: " << std::to_string(state.pos_x) << std::endl;        // field of struct (state)
  os << " Pos_y: " << std::to_string(state.pos_y) << std::endl;        // field of struct (state)
  os << " Yaw: " << std::to_string(state.yaw) << std::endl;            // field of struct (state)
  os << " V_x: " << std::to_string(state.vel_x) << std::endl;          // field of struct (state)
  os << " V_y: " << std::to_string(state.vel_y) << std::endl;          // field of struct (state)
  os << " Yaw_rate: " << std::to_string(state.yaw_rate) << std::endl;  // field of struct (state)
  return os;
}

}  // namespace custom_model
}  // namespace crs_models
#endif
