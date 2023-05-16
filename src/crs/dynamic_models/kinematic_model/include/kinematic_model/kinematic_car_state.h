#ifndef KINEMATIC_MODEL_KINEMATIC_CAR_STATE_H
#define KINEMATIC_MODEL_KINEMATIC_CAR_STATE_H

#include <iostream>

namespace crs_models
{
namespace kinematic_model
{

struct kinematic_car_state
{
  double pos_x;
  double pos_y;
  double yaw;
  double velocity;
  kinematic_car_state() : pos_x(0), pos_y(0), yaw(0), velocity(0){};  // default constructor
  kinematic_car_state(double pos_x, double pos_y, double yaw, double velocity)
    : pos_x(pos_x), pos_y(pos_y), yaw(yaw), velocity(velocity){};  // default constructor
};

inline kinematic_car_state operator+(const kinematic_car_state& a, const kinematic_car_state& b)
{
  kinematic_car_state added_struct;

  added_struct.pos_x = a.pos_x + b.pos_x;
  added_struct.pos_y = a.pos_y + b.pos_y;
  added_struct.yaw = a.yaw + b.yaw;
  added_struct.velocity = a.velocity + b.velocity;
  return added_struct;
}

inline kinematic_car_state operator*(double b, const kinematic_car_state& a)
{
  kinematic_car_state product_struct;
  product_struct.pos_x = a.pos_x * b;
  product_struct.pos_y = a.pos_y * b;
  product_struct.yaw = a.yaw * b;
  product_struct.velocity = a.velocity * b;
  return product_struct;
}

inline void operator+=(kinematic_car_state& a, const kinematic_car_state& b)
{
  a.pos_x = a.pos_x + b.pos_x;
  a.pos_y = a.pos_y + b.pos_y;
  a.yaw = a.yaw + b.yaw;
  a.velocity = a.velocity + b.velocity;
}

inline bool operator==(const kinematic_car_state& a, const kinematic_car_state& b)
{
  return (a.pos_x == b.pos_x) && (a.pos_y == b.pos_y) && (a.yaw == b.yaw) && (a.velocity == b.velocity);
}

inline std::ostream& operator<<(std::ostream& os,
                                const kinematic_car_state& state)  // how to print struct (state) to output stream (os)
{
  os << "kinematic_car_state:" << std::endl;                           // Type of struct (state)
  os << " Pos_x: " << std::to_string(state.pos_x) << std::endl;        // field of struct (state)
  os << " Pos_y: " << std::to_string(state.pos_y) << std::endl;        // field of struct (state)
  os << " Yaw: " << std::to_string(state.yaw) << std::endl;            // field of struct (state)
  os << " Velocity: " << std::to_string(state.velocity) << std::endl;  // field of struct (state)
  return os;
}

}  // namespace kinematic_model
}  // namespace crs_models
#endif
