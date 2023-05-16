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

int NUM_ESTIMATES = 500;
int NUM_TRANSIENT = 10;

double X_PRECISION = 0.02;
double Y_PRECISION = 0.03;
double YAW_PRECISION = 0.03;
double VX_B_PRECISION = 0.03;
double VY_B_PRECISION = 0.03;
double DYAW_PRECISION = 0.6;

std::shared_ptr<ros::NodeHandle> nh;
std::vector<std::pair<crs_msgs::car_state_cart, crs_msgs::car_state_cart>> states;

void measurementCallback(const crs_msgs::car_state_cart::ConstPtr& state_gt,
                         const crs_msgs::car_state_cart::ConstPtr& state_est)
{
  states.push_back(std::make_pair<>(*state_gt, *state_est));
}

/**
 * @brief Checks if estimated states and ground truth states are the same
 *
 */
TEST(ClosedLoopTestSuite, testEKF)
{
  // get current gt and state estimate
  message_filters::Subscriber<crs_msgs::car_state_cart> state_gt_sub(*nh, "ros_simulator/gt_state", 10);
  message_filters::Subscriber<crs_msgs::car_state_cart> state_est_sub(*nh, "estimation_node/best_state", 10);
  // Want gt and estimate at same time.
  typedef message_filters::sync_policies::ApproximateTime<crs_msgs::car_state_cart, crs_msgs::car_state_cart>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), state_gt_sub, state_est_sub);
  sync.registerCallback(boost::bind(&measurementCallback, _1, _2));

  // wait during test
  while (states.size() < NUM_ESTIMATES)
  {
    ros::spinOnce();  // makes ROS process callbacks
    ros::Rate(100).sleep();
  }

  float squared_error[6] = { 0.0 };

  for (int i = NUM_TRANSIENT - 1; i < states.size(); i++)
  {
    auto input = states[i];
    squared_error[0] += std::pow(input.first.x - input.second.x, 2);
    squared_error[1] += std::pow(input.first.y - input.second.y, 2);
    squared_error[2] += std::pow(input.first.yaw - input.second.yaw, 2);
    squared_error[3] += std::pow(input.first.vx_b - input.second.vx_b, 2);
    squared_error[4] += std::pow(input.first.vy_b - input.second.vy_b, 2);
    squared_error[5] += std::pow(input.first.dyaw - input.second.dyaw, 2);
  }

  float root_mean_squared_error[6] = { 0.0 };
  root_mean_squared_error[0] = std::sqrt(squared_error[0] / (NUM_ESTIMATES - NUM_TRANSIENT));
  root_mean_squared_error[1] = std::sqrt(squared_error[1] / (NUM_ESTIMATES - NUM_TRANSIENT));
  root_mean_squared_error[2] = std::sqrt(squared_error[2] / (NUM_ESTIMATES - NUM_TRANSIENT));
  root_mean_squared_error[3] = std::sqrt(squared_error[3] / (NUM_ESTIMATES - NUM_TRANSIENT));
  root_mean_squared_error[4] = std::sqrt(squared_error[4] / (NUM_ESTIMATES - NUM_TRANSIENT));
  root_mean_squared_error[5] = std::sqrt(squared_error[5] / (NUM_ESTIMATES - NUM_TRANSIENT));

  // Use error stream instead of info stream since info stream does not work in tests
  ROS_ERROR_STREAM("EKF estimator test: absolute root mean squared error x: " << root_mean_squared_error[0]);
  ROS_ERROR_STREAM("EKF estimator test: absolute root mean squared error y: " << root_mean_squared_error[1]);
  ROS_ERROR_STREAM("EKF estimator test: absolute root mean squared error yaw: " << root_mean_squared_error[2]);
  ROS_ERROR_STREAM("EKF estimator test: absolute root mean squared error vx_b: " << root_mean_squared_error[3]);
  ROS_ERROR_STREAM("EKF estimator test: absolute root mean squared error vy_b: " << root_mean_squared_error[4]);
  ROS_ERROR_STREAM("EKF estimator test: absolute root mean squared error dyaw: " << root_mean_squared_error[5]);
  ROS_ERROR_STREAM("DYAW " << DYAW_PRECISION << "  " << nh->getNamespace());

  // Check square norm of error vector
  EXPECT_LT(root_mean_squared_error[0], X_PRECISION);
  EXPECT_LT(root_mean_squared_error[1], Y_PRECISION);
  EXPECT_LT(root_mean_squared_error[2], YAW_PRECISION);
  EXPECT_LT(root_mean_squared_error[3], VX_B_PRECISION);
  EXPECT_LT(root_mean_squared_error[4], VY_B_PRECISION);
  EXPECT_LT(root_mean_squared_error[5], DYAW_PRECISION);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "closed_loop_estimator_test");
  nh.reset(new ros::NodeHandle(""));

  nh->getParam("n_estimates", NUM_ESTIMATES);
  nh->getParam("n_transient", NUM_TRANSIENT);

  nh->getParam("x_precision", X_PRECISION);
  nh->getParam("y_precision", Y_PRECISION);
  nh->getParam("yaw_precision", YAW_PRECISION);
  nh->getParam("vx_precision", VX_B_PRECISION);
  nh->getParam("vy_precision", VY_B_PRECISION);
  nh->getParam("dyaw_precision", DYAW_PRECISION);

  testing::InitGoogleTest(&argc, argv);

  ros::Duration(5).sleep();  // Wait for ekf to start....

  return RUN_ALL_TESTS();
}
