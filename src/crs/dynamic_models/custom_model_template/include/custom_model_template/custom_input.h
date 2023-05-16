#ifndef CUSTOM_MODEL_TEMPLATE_CUSTOM_INPUT_H
#define CUSTOM_MODEL_TEMPLATE_CUSTOM_INPUT_H

#include <iostream>

namespace crs_models
{
namespace custom_model
{

struct custom_input
{
  double torque;
  double steer;
  custom_input() : torque(0), steer(0){};                                      // default constructor
  custom_input(double torque, double steer) : torque(torque), steer(steer){};  // Constructor
};

inline custom_input operator+(const custom_input& a, const custom_input& b)
{
  custom_input added_struct;

  added_struct.steer = a.steer + b.steer;
  added_struct.torque = a.torque + b.torque;

  return added_struct;
}

inline void operator+=(custom_input& a, const custom_input& b)
{
  a.steer = a.steer + b.steer;
  a.torque = a.torque + b.torque;
}

inline bool operator==(const custom_input& a, const custom_input& b)
{
  return (a.steer == b.steer) && (a.torque == b.torque);
}

inline std::ostream& operator<<(std::ostream& os,
                                const custom_input& input)  // how to print struct (input) to output stream (os)
{
  os << "model_input:" << std::endl;                               // Type of struct (input)
  os << " Torque: " << std::to_string(input.torque) << std::endl;  // field of struct (input)
  os << " Steer: " << std::to_string(input.steer) << std::endl;    // field of struct (input)
  return os;
}

}  // namespace custom_model
}  // namespace crs_models
#endif
