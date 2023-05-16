#include "ros_controllers/dynamic_config.h"

namespace ros_controllers
{
#ifdef pid_controller_FOUND
DynamicPIDConfigServer::DynamicPIDConfigServer(std::shared_ptr<crs_controls::PacejkaPIDController> controller)
  : controller_(controller)
{
  f = boost::bind(&DynamicPIDConfigServer::callback, this, _1, _2);
  server.setCallback(f);
};

void DynamicPIDConfigServer::callback(ros_controllers::PIDConfig& config, uint32_t level)
{
  if (!controller_)
    return;
  if (!ignored_first_call)
  {  // Ignore first call. This lets us set default values using the parameter server and not the .cfg file
    ignored_first_call = true;
    return;
  }

  crs_controls::pid_config pid_cfg = controller_->getConfig();
  pid_cfg.Kd = config.Kd;
  pid_cfg.Kp = config.Kp;
  pid_cfg.Ki = config.Ki;
  pid_cfg.use_filter = config.use_filter;
  pid_cfg.target_velocity = config.target_velocity;
  pid_cfg.lag_compensation_time = config.lag_compensation_time;
  controller_->setConfig(pid_cfg);
};
#endif

#ifdef ff_fb_controller_FOUND
DynamicFfFbConfigServer::DynamicFfFbConfigServer(std::shared_ptr<crs_controls::FfFbController> controller)
  : controller_(controller)
{
  f = boost::bind(&DynamicFfFbConfigServer::callback, this, _1, _2);
  server.setCallback(f);
};

void DynamicFfFbConfigServer::callback(ros_controllers::ff_fbConfig& config, uint32_t level)
{
  if (!controller_)
    return;
  if (!ignored_first_call)
  {  // Ignore first call. This lets us set default values using the parameter server and not the .cfg file
    ignored_first_call = true;
    return;
  }

  crs_controls::FfFbConfig pid_cfg = controller_->getConfig();
  pid_cfg.Kd = config.Kd;
  pid_cfg.Kp = config.Kp;
  pid_cfg.Ki = config.Ki;
  pid_cfg.target_velocity = config.target_velocity;
  pid_cfg.K_torque_curv = config.K_torque_curv;
  pid_cfg.lag_compensation_time = config.lag_compensation_time;
  pid_cfg.mean_curv_dist = config.mean_curv_dist;
  pid_cfg.use_filter = config.use_filter;
  controller_->setConfig(pid_cfg);
};
#endif

#ifdef mpc_controller_FOUND
DynamicPacejkaMPCCConfigServer::DynamicPacejkaMPCCConfigServer(
    std::shared_ptr<crs_controls::PacejkaMpccController> controller)
  : controller_(controller)
{
  f = boost::bind(&DynamicPacejkaMPCCConfigServer::callback, this, _1, _2);
  server.setCallback(f);
};

void DynamicPacejkaMPCCConfigServer::callback(ros_controllers::pacejka_mpccConfig& config, uint32_t level)
{
  if (!controller_)
    return;
  if (!ignored_first_call)
  {  // Ignore first call. This lets us set default values using the parameter server and not the .cfg file
    ignored_first_call = true;
    return;
  }

  crs_controls::mpcc_pacejka_config pid_cfg = controller_->getConfig();
  pid_cfg.Q1 = config.Q1;
  pid_cfg.Q2 = config.Q2;
  pid_cfg.R1 = config.R1;
  pid_cfg.R2 = config.R2;
  pid_cfg.R3 = config.R3;
  pid_cfg.q = config.q;
  pid_cfg.lag_compensation_time = config.lag_compensation_time;
  controller_->setConfig(pid_cfg);
};
#endif
}  // namespace ros_controllers
