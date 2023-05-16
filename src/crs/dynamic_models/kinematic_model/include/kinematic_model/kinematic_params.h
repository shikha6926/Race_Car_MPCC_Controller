#ifndef KINEMATIC_MODEL_KINEMATIC_PARAMS_H
#define KINEMATIC_MODEL_KINEMATIC_PARAMS_H

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace crs_models
{
namespace kinematic_model
{
struct kinematic_params
{
  // car params
  double lr;
  double lf;
  double tau;

  // torque mapping
  double a;
  double b;
};

inline std::ostream& operator<<(std::ostream& os,
                                const kinematic_params& params)  // how to print struct (state) to output stream (os)
{
  os << "kinematic_params:" << std::endl;
  os << "lr: " << std::to_string(params.lr) << std::endl;
  os << "lf: " << std::to_string(params.lf) << std::endl;
  os << "tau: " << std::to_string(params.tau) << std::endl;
  os << "a: " << std::to_string(params.a) << std::endl;
  os << "b: " << std::to_string(params.b) << std::endl;
  return os;
}

inline bool loadParamsFromFile(std::string file_path, kinematic_params& params)
{
  try
  {
    YAML::Node file_content = YAML::LoadFile(file_path);
    params.lr = file_content["lr"].as<double>();
    params.lf = file_content["lf"].as<double>();
    params.tau = file_content["tau"].as<double>();
    params.a = file_content["a"].as<double>();
    params.b = file_content["b"].as<double>();
  }
  catch (YAML::BadFile& e)
  {
    std::cout << "Error reading file at " << file_path << e.msg << std::endl;
    return false;
  }
  return true;
}
}  // namespace kinematic_model
}  // namespace crs_models
#endif
