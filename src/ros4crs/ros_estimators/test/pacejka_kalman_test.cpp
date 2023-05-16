#include <gtest/gtest.h>
#include <ros/ros.h>

#include <crs_msgs/car_input.h>
#include <crs_msgs/car_state_cart.h>
#include <geometry_msgs/TransformStamped.h>

std::shared_ptr<ros::NodeHandle> nh;

ros::Publisher input_publisher;
ros::Publisher measurement_publisher;

void publish_input_msg()
{
  crs_msgs::car_input input;
  input.header.stamp = ros::Time::now();
  input.steer = 0;
  input.torque = 0.5;
  input_publisher.publish(input);
}

void publish_meas_msg()
{
  geometry_msgs::TransformStamped vicon_meas;
  vicon_meas.header.stamp = ros::Time::now();
  vicon_meas.transform.translation.x = vicon_meas.transform.translation.x + 0.5;
  measurement_publisher.publish(vicon_meas);
}

/**
 * @brief Test inputCallback of ekf. Publish a control input and check if the published state changes.
 *
 */
TEST(KalmanTestSuite, inputCallbackTest)
{
  // Current state estimate
  auto state_msg = ros::topic::waitForMessage<crs_msgs::car_state_cart>("/estimation_node/best_state");
  // publish input
  publish_input_msg();
  // wait a little
  ros::spinOnce();
  ros::Duration(0.5).sleep();  // sleep for 0.2s
  // publish input
  publish_input_msg();
  // wait a little
  ros::Duration(0.2).sleep();  // sleep for 0.2s

  ros::spinOnce();
  ros::Duration(0.5).sleep();  // sleep for 0.2s
  // publish input
  publish_input_msg();

  auto new_state_msg = ros::topic::waitForMessage<crs_msgs::car_state_cart>("/estimation_node/best_state");
  EXPECT_NE(new_state_msg->x, state_msg->x);
}

/**
 * @brief Test viconMeasurementCallback of the EKF. Publish a measurement input and check if the published state
 * changes.
 *
 */
TEST(KalmanTestSuite, viconMeasurementCallback)
{
  // Current state estimate
  auto state_msg = ros::topic::waitForMessage<crs_msgs::car_state_cart>("/estimation_node/best_state");
  // publish input
  publish_meas_msg();
  // wait a little
  ros::Duration(0.2).sleep();  // sleep for 0.2s
  // publish input
  publish_meas_msg();
  // wait a little
  ros::Duration(0.2).sleep();  // sleep for 0.2s

  auto new_state_msg = ros::topic::waitForMessage<crs_msgs::car_state_cart>("/estimation_node/best_state");
  EXPECT_NE(new_state_msg->x, state_msg->x);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalman_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);

  ros::Duration(5).sleep();  // Wait for ekf to start....
  input_publisher = nh->advertise<crs_msgs::car_input>("/ros_simulator/control_input", 10);
  measurement_publisher = nh->advertise<geometry_msgs::TransformStamped>("/ros_simulator/vicon", 10);

  return RUN_ALL_TESTS();
}
