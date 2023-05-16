#include "commons/static_track_trajectory.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <iostream>
#include <random>

// For random number generation
std::random_device rd;
std::default_random_engine eng(rd());  // NOLINT
std::uniform_real_distribution<float> distr(0.0, 1.0);

float getRandomFloat(float lower_limit, float upper_limit)
{
  return (distr(rd) * (upper_limit - lower_limit)) + lower_limit;
}

/**
 * @brief check if the center line of a generated track is correct
 */
TEST(TrackTest, testTrackGeneration)
{
  std::vector<double> x_cords;
  std::vector<double> y_cords;
  std::vector<Eigen::Vector2d> pts;
  for (int i = 0; i < 100; i++)
  {
    auto x = getRandomFloat(-100, 100);
    auto y = getRandomFloat(-100, 100);
    x_cords.push_back(x);
    y_cords.push_back(y);
    pts.push_back(Eigen::Vector2d(x, y));
  }

  crs_controls::StaticTrackTrajectory track_from_list(x_cords, y_cords);
  // Make sure tracks are equal
  auto center_line = track_from_list.getCenterLine();
  EXPECT_EQ(center_line.size(), x_cords.size());

  for (int i = 0; i < center_line.size(); i++)
  {
    EXPECT_EQ(center_line[i].x(), x_cords[i]);
    EXPECT_EQ(center_line[i].y(), y_cords[i]);
  }

  crs_controls::StaticTrackTrajectory track_from_pts(pts);
  center_line = track_from_pts.getCenterLine();
  EXPECT_EQ(center_line.size(), x_cords.size());

  for (int i = 0; i < center_line.size(); i++)
  {
    EXPECT_EQ(center_line[i].x(), x_cords[i]);
    EXPECT_EQ(center_line[i].y(), y_cords[i]);
  }
}

/**
 * @brief Check if the closest point function works when directly requesting a point on the track
 */
TEST(TrackTest, testClosestPointRandomEqual)
{
  std::vector<double> x_cords;
  std::vector<double> y_cords;
  std::vector<Eigen::Vector2d> pts;

  for (int i = 0; i < 100; i++)
  {
    auto x = i + getRandomFloat(-0.4, 0.4);  // NOLINT
    auto y = i + getRandomFloat(-0.4, 0.4);  // NOLINT
    x_cords.push_back(x);
    y_cords.push_back(y);
    pts.push_back(Eigen::Vector2d(x, y));
  }

  crs_controls::StaticTrackTrajectory track(x_cords, y_cords);

  // Check query point is closest
  for (int i = 0; i < 100; i++)
  {
    auto closest_pt = track.getClosestTrackPoint(pts[i], 1);
    EXPECT_EQ(pts[i].x(), closest_pt.x());
    EXPECT_EQ(pts[i].y(), closest_pt.y());
  }
}

/**
 * @brief Check if the closest point function works when the track is a line and requesting a point next to the track
 *
 */
TEST(TrackTest, testClosestPoint1DLine)
{
  for (const int precision : { 1, 2, 8, 16 })
  {
    std::vector<Eigen::Vector2d> pts;

    // Create 1D line with step size 1
    for (int i = 0; i < 100; i++)
    {
      auto x = i;
      auto y = 0;
      pts.push_back(Eigen::Vector2d(x, y));
    }

    crs_controls::StaticTrackTrajectory track(pts);

    // Make sure y offsets do not matter for closest point query
    for (int i = 0; i < 100; i++)
    {
      Eigen::Vector2d query_point(pts[i].x(), getRandomFloat(-10, 10));
      auto closest_pt = track.getClosestTrackPoint(query_point, precision);
      EXPECT_EQ(pts[i].x(), closest_pt.x());
      EXPECT_EQ(pts[i].y(), closest_pt.y());
    }

    // Make sure closest point is selected when putting noise on x axis
    for (int i = 0; i < 100; i++)
    {
      auto x_noise = getRandomFloat(-1, 0);
      int closest_idx = i;
      if (x_noise < -0.5 && i != 0)
        closest_idx = i - 1;

      Eigen::Vector2d query_point(pts[i].x() + x_noise, 0);
      auto closest_pt = track.getClosestTrackPoint(query_point, precision);
      EXPECT_EQ(pts[closest_idx].x(), closest_pt.x());
      EXPECT_EQ(pts[closest_idx].y(), closest_pt.y());
    }
  }
}

/**
 * @brief Check if the lateral error calculation and side calculation works correctly if the track is a 1d line
 *
 */
TEST(TrackTest, testErrorCalculation)
{
  std::vector<Eigen::Vector2d> pts;

  // Create 1D line with step size 1
  for (int i = 0; i < 100; i++)
  {
    auto x = i;
    auto y = 0;
    pts.push_back(Eigen::Vector2d(x, y));
  }

  crs_controls::StaticTrackTrajectory track(pts);
  // Make sure errors are correct
  for (int i = 0; i < 99; i++)
  {
    double noise_y = getRandomFloat(-10, 10);
    Eigen::Vector2d query_point(pts[i].x(), noise_y);
    auto info = track.getTrackError(query_point, 16);
    double side = info.side;
    double dist = info.lateral_error;
    EXPECT_EQ(side > 0, noise_y < 0);
    EXPECT_EQ(dist, std::abs(noise_y));
  }
}
int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}