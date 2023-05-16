#ifndef PACEJKA_MODEL_PACEJKA_CAR_INPUT_H
#define PACEJKA_MODEL_PACEJKA_CAR_INPUT_H

#include <iostream>

namespace crs_models
{
namespace pacejka_model
{

struct pacejka_car_input
{
  double torque;
  double steer;
  pacejka_car_input() : torque(0), steer(0){};                                      // default constructor
  pacejka_car_input(double torque, double steer) : torque(torque), steer(steer){};  // Constructor
};

inline pacejka_car_input operator+(const pacejka_car_input& a, const pacejka_car_input& b)
{
  pacejka_car_input added_struct;

  added_struct.steer = a.steer + b.steer;
  added_struct.torque = a.torque + b.torque;

  return added_struct;
}

inline void operator+=(pacejka_car_input& a, const pacejka_car_input& b)
{
  a.steer = a.steer + b.steer;
  a.torque = a.torque + b.torque;
}

inline bool operator==(const pacejka_car_input& a, const pacejka_car_input& b)
{
  return (a.steer == b.steer) && (a.torque == b.torque);
}

inline std::ostream& operator<<(std::ostream& os,
                                const pacejka_car_input& input)  // how to print struct (input) to output stream (os)
{
  os << "model_input:" << std::endl;                               // Type of struct (input)
  os << " Torque: " << std::to_string(input.torque) << std::endl;  // field of struct (input)
  os << " Steer: " << std::to_string(input.steer) << std::endl;    // field of struct (input)
  return os;
}

}  // namespace pacejka_model
}  // namespace crs_models
#endif
