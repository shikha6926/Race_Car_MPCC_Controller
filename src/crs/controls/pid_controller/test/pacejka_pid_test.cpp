#include <gtest/gtest.h>
#include <Eigen/Core>
#include <commons/static_track_trajectory.h>
#include <iostream>
#include <random>

#include "pid_controller/pacejka_pid_controller.h"

// For random number generation
std::random_device rd;
std::default_random_engine eng(rd());  // NOLINT
std::uniform_real_distribution<float> distr(0.0, 1.0);

float getRandomFloat(float lower_limit, float upper_limit)
{
  return (distr(rd) * (upper_limit - lower_limit)) + lower_limit;
}

/**
 * @brief Checks if the PID controller steers towards the center of the track
 */
TEST(TrackTest, testSteerInputTowardsCenterLine)
{
  std::vector<Eigen::Vector2d> pts;
  // Create 1D line with step size 1
  for (int i = 0; i < 100; i++)
  {
    auto x = i;
    auto y = 0;
    pts.push_back(Eigen::Vector2d(x, y));
  }

  auto linear_line_track = std::make_shared<crs_controls::StaticTrackTrajectory>(pts);
  for (int RUN = 0; RUN < 100; RUN++)
  {
    double target_vel = getRandomFloat(0.2, 1.2);
    crs_controls::pid_config p_ctrl_config;
    p_ctrl_config.target_velocity = target_vel;
    p_ctrl_config.lag_compensation_time = 0;
    p_ctrl_config.use_filter = 0;
    p_ctrl_config.a_torque = 1;
    p_ctrl_config.b_torque = 0;
    p_ctrl_config.Kp = 1;
    p_ctrl_config.Kd = 0;
    p_ctrl_config.Ki = 0;
    p_ctrl_config.steer_limit = 0.4;

    crs_controls::PacejkaPIDController p_controller(p_ctrl_config, linear_line_track);

    // chose random point
    int idx = int(getRandomFloat(0, 99));
    //
    auto car_position = linear_line_track->operator[](idx);
    float n_y = getRandomFloat(-1, 1);
    float n_x = getRandomFloat(0, 0.5);
    Eigen::Vector2d perturbed_position(car_position.x() + n_x, n_y);

    crs_models::pacejka_model::pacejka_car_state state = {
      perturbed_position.x(), perturbed_position.y(), 0, 0.0, 0, 0
    };
    auto ctrl_input = p_controller.getControlInput(state);

    if ((n_y > 0) != (ctrl_input.steer < 0))
      std::cout << n_x << " " << n_y << "   " << ctrl_input << std::endl;
    EXPECT_EQ(n_y > 0, ctrl_input.steer < 0);                 // Make sure control input would turn towards center line
    EXPECT_EQ(ctrl_input.torque, std::min(target_vel, 1.0));  // Make sure control input would turn towards center line
  }
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}