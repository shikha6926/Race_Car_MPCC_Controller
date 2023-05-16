#ifndef KINEMATIC_MODEL_KINEMATIC_CAR_INPUT_H
#define KINEMATIC_MODEL_KINEMATIC_CAR_INPUT_H

#include <iostream>

namespace crs_models
{
namespace kinematic_model
{

struct kinematic_car_input
{
  double torque;
  double steer;
  kinematic_car_input() : torque(0), steer(0){};                                      // default constructor
  kinematic_car_input(double torque, double steer) : torque(torque), steer(steer){};  // Constructor
};

inline kinematic_car_input operator+(const kinematic_car_input& a, const kinematic_car_input& b)
{
  kinematic_car_input added_struct;

  added_struct.steer = a.steer + b.steer;
  added_struct.torque = a.torque + b.torque;

  return added_struct;
}

inline void operator+=(kinematic_car_input& a, const kinematic_car_input& b)
{
  a.steer = a.steer + b.steer;
  a.torque = a.torque + b.torque;
}

inline bool operator==(const kinematic_car_input& a, const kinematic_car_input& b)
{
  return (a.steer == b.steer) && (a.torque == b.torque);
}

inline std::ostream& operator<<(std::ostream& os,
                                const kinematic_car_input& input)  // how to print struct (input) to output stream (os)
{
  os << "model_input:" << std::endl;                               // Type of struct (input)
  os << " Torque: " << std::to_string(input.torque) << std::endl;  // field of struct (input)
  os << " Steer: " << std::to_string(input.steer) << std::endl;    // field of struct (input)
  return os;
}

}  // namespace kinematic_model
}  // namespace crs_models
#endif
