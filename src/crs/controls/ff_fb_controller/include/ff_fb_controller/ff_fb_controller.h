#ifndef SRC_CRS_CONTROLS_FF_FB_CONTROLLER_INCLUDE_FF_FB_CONTROLLER_FF_FB_CONTROLLER
#define SRC_CRS_CONTROLS_FF_FB_CONTROLLER_INCLUDE_FF_FB_CONTROLLER_FF_FB_CONTROLLER

#include "ff_fb_config.h"
#include <commons/filter.h>
#include <controls/model_based_controller.h>
#include <memory>
#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_params.h>

namespace crs_controls
{
class FfFbController
  : public ModelBasedController<crs_models::pacejka_model::pacejka_params, crs_models::pacejka_model::pacejka_car_state,
                                crs_models::pacejka_model::pacejka_car_input>
{
private:
  // FfFbConfig Config
  FfFbConfig config_;

  // Errors
  double pos_err_ = 0;
  double integral_err_ = 0;
  double prev_pos_err_ = 0;  // Used to approximate derivative
  double prev_yaw_ = 0;
  double prev_err_angle_ = 0;
  double prev_track_angle_ = 0;

  // Filter used to filter steer angle
  Filter u_steer_filter_;

  std::shared_ptr<StaticTrackTrajectory> getStaticTrack();

public:
  // Constructor
  FfFbController(FfFbConfig config, std::shared_ptr<crs_models::pacejka_model::pacejka_params> model,
                 std::shared_ptr<StaticTrackTrajectory> track);

  /**
   * Executes FF FB controller, returning a new input to apply
   */
  crs_models::pacejka_model::pacejka_car_input
  getControlInput(crs_models::pacejka_model::pacejka_car_state state,
                  double timestamp = 0 /* timestamp will be ignored */) override;

  /**
   *  Sets the internal config
   *  Allows e.g. to update PID gains during runtime, or change target velocity.
   */
  void setConfig(FfFbConfig config);

  FfFbConfig& getConfig()
  {
    return config_;
  }

  /**
   * @brief Returns wether the controller is currently initializing
   *
   * @return true if controller is initializing
   * @return false if controller is not initializing
   */
  const bool isInitializing() override
  {
    return false;  // FF_FB does not need initializing
  }
};

}  // namespace crs_controls

#endif /* SRC_CRS_CONTROLS_FF_FB_CONTROLLER_INCLUDE_FF_FB_CONTROLLER_FF_FB_CONTROLLER */
