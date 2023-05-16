/**
 * @file    wifi_com_node.cpp
 * @author  Lukas Vogel
 * @brief   Node that instantiates the WifiCom object.
 */

#include "ros/ros.h"
#include "crs_msgs/car_com.h"
#include "std_msgs/String.h"
#include "wifi_com/WiFiCom.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wifi_com");
  ros::NodeHandle n;

  // The wifi_com_node should run at 500Hz, double the rate that packets will
  // arrive. This way, incoming packets should not suffer from additional large
  // delays (=> 1-2ms added by the polling period)
  ros::Rate rate(500);

  WiFiCom com(n);

  while (ros::ok())
  {
    // process all incoming messages first, this will send outgoing packets
    ros::spinOnce();
    // poll socket for incoming packets and publish the data as message
    com.poll();
    rate.sleep();
  }

  return 0;
}
