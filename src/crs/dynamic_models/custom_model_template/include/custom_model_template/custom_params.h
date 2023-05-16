#ifndef CUSTOM_MODEL_TEMPLATE_CUSTOM_PARAMS_H
#define CUSTOM_MODEL_TEMPLATE_CUSTOM_PARAMS_H

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace crs_models
{
namespace custom_model
{

struct custom_params
{
  // car params
  double lr;
  double lf;
  double m;
  double I;
  // lateral force params
  double Df;
  double Cf;
  double Bf;
  double Dr;
  double Cr;
  double Br;
  // longitudinal force params
  double Cm1;
  double Cm2;
  double Cd;
  double Croll;
};

inline std::ostream& operator<<(std::ostream& os,
                                const custom_params& params)  // how to print struct (state) to output stream (os)
{
  os << "pacejka_params:" << std::endl;
  os << "lr: " << std::to_string(params.lr) << std::endl;
  os << "lf: " << std::to_string(params.lf) << std::endl;
  os << "m: " << std::to_string(params.m) << std::endl;
  os << "I: " << std::to_string(params.I) << std::endl;
  os << "Df: " << std::to_string(params.Df) << std::endl;
  os << "Cf: " << std::to_string(params.Cf) << std::endl;
  os << "Bf: " << std::to_string(params.Bf) << std::endl;
  os << "Dr: " << std::to_string(params.Dr) << std::endl;
  os << "Cr: " << std::to_string(params.Cr) << std::endl;
  os << "Br: " << std::to_string(params.Br) << std::endl;
  os << "Cm1: " << std::to_string(params.Cm1) << std::endl;
  os << "Cm2: " << std::to_string(params.Cm2) << std::endl;
  os << "Cd: " << std::to_string(params.Cd) << std::endl;
  os << "Croll: " << std::to_string(params.Croll) << std::endl;
  return os;
}

inline bool loadParamsFromFile(std::string file_path, custom_params& params)
{
  try
  {
    YAML::Node file_content = YAML::LoadFile(file_path);
    params.lr = file_content["lr"].as<double>();
    params.lf = file_content["lf"].as<double>();
    params.m = file_content["m"].as<double>();
    params.I = file_content["I"].as<double>();
    params.Df = file_content["Df"].as<double>();
    params.Cf = file_content["Cf"].as<double>();
    params.Bf = file_content["Bf"].as<double>();
    params.Dr = file_content["Dr"].as<double>();
    params.Cr = file_content["Cr"].as<double>();
    params.Br = file_content["Br"].as<double>();
    params.Cm1 = file_content["Cm1"].as<double>();
    params.Cm2 = file_content["Cm2"].as<double>();
    params.Cd = file_content["Cd"].as<double>();
    params.Croll = file_content["Croll"].as<double>();
  }
  catch (YAML::BadFile& e)
  {
    std::cout << "Error reading file at " << file_path << e.msg << std::endl;
    return false;
  }
  return true;
}
}  // namespace custom_model
}  // namespace crs_models
#endif
