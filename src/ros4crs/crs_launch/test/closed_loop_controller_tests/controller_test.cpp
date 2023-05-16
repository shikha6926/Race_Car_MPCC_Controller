#include <gtest/gtest.h>
#include <ros/ros.h>

#include <crs_msgs/car_input.h>
#include <crs_msgs/car_state_cart.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <functional>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <ros_crs_utils/parameter_io.h>

int NUM_STATES = 200;  // over how many state messages the error is calculated

double MEAN_DISTANCE_PRECISION = 0.1;  // mean distances from track center line

std::shared_ptr<ros::NodeHandle> nh;
std::vector<crs_msgs::car_state_cart> states;

/**
 * @brief Checks if car follows centerline.
 *
 */
TEST(ClosedLoopTestSuite, testController)
{
  // wait during test
  while (states.size() < NUM_STATES)
  {
    // Current state estimate
    auto state_msg = ros::topic::waitForMessage<crs_msgs::car_state_cart>("ros_simulator/gt_state");
    states.push_back(*state_msg);
  }

  // evaluate
  float mean_distance_error = 0.0;
  auto trackMgr = parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle("track"));  // contains track inforamtion

  for (int i = 0; i < states.size(); i++)
  {
    auto error_info = trackMgr->getTrackError(Eigen::Vector2d(states[i].x, states[i].y));  // get lateral error of state
                                                                                           // to track center
    mean_distance_error += error_info.lateral_error / states.size();
  }

  // Use error stream instead of info stream since info stream does not work in tests. UGLY
  ROS_ERROR_STREAM("Controllers test: average error from track: " << mean_distance_error);
  EXPECT_LT(mean_distance_error, MEAN_DISTANCE_PRECISION);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "closed_controller_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);

  nh->getParam("num_states", NUM_STATES);
  nh->getParam("mean_distance_precision", MEAN_DISTANCE_PRECISION);

  ros::Duration(5).sleep();  // Wait for ekf to start....

  return RUN_ALL_TESTS();
}
