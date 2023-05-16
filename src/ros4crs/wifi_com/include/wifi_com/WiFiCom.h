#ifndef WIFI_COM_WIFICOM_H
#define WIFI_COM_WIFICOM_H

/**
 * @file WiFiCom.h
 * @author Lukas Vogel (vogellu@ethz.ch)
 * @brief Class that communicates with cars over Wi-Fi and UDP.
 */

#pragma once

#include <sys/socket.h>

#include "IMUMeasurement.pb.h"
#include "MotorInput.pb.h"
#include "SingleControlInput.pb.h"
#include "SteeringPositionMeasurement.pb.h"
#include "Lighthouse.pb.h"
#include "crs_msgs/car_input.h"
#include "ros/ros.h"
#include "wifi_com/UDPServer.h"

/* Number of supported Lighthouse base stations */
#define LIGHTHOUSE_BASE_STATIONS 16

/**
 * @brief Class that communicates with cars over Wi-Fi and UDP.
 *
 * Let ns be the namespace of this node. Then the WiFiCom class subscribes to
 * the /ns/control_input topic and sends the torque and steer commands it
 * receives to the car. The cars send the following data back:
 *
 * - IMU data, which is published on /ns/imu;
 * - the steer position, which is published on /ns/car_steer_state;
 * - the low-level control inputs that the car gave for logging/debugging, which
 *    is published on /ns/car_ll_control_input.
 *
 * The WiFiCom node communicates with the cars via UDP sockets and protocol
 * buffers. To ensure that incoming messages are processed in a timely manner,
 * the socket needs to be polled for new data arriving. This can be achieved by
 * calling the ::poll() method, which should happen at least at twice the rate
 * of incoming packets, so for 250Hz of data from the car, 500Hz polling should
 * be appropriate.
 *
 * Internally, the WiFiCom class uses Google Protocol Buffers for communication
 * with the car. The messages that are sent are defined in ./msgs/proto/ and are
 * compiled into C++ header/source files that deal with the serialization.
 * To add/modify messages, refer to the documentation of the protobuf:
 * https://developers.google.com/protocol-buffers/
 */
class WiFiCom
{
public:
  /**
   * @brief Constructor
   * @param n nodeHandle
   */
  WiFiCom(ros::NodeHandle& n);

  /**
   * @brief Destructor
   */
  ~WiFiCom();

  /**
   * @brief Polls the socket to see if any data was received.
   *
   * The UDP socket does not interrupt the process if new data is received.
   * There need to be periodic checks if new data has arrived, since this node
   * does not run in a multi-threaded manner and can't afford to block on
   * waiting for data.
   */
  void poll();

  /**
   * @brief Callback for incoming /ns/control_input messages.
   *
   * Every /ns/control_input message gets sent to the UDP socket.
   * @param msg the input
   */
  void controllerCallback(const crs_msgs::car_input::ConstPtr& msg);

private:
  /* Private methods -------------------------------------------------------- */
  /** Load the parameters from the configuration file */
  void loadParameters();

  /** Set up all subscriber objects. */
  void setupSubscribers();

  /** Set up all publisher objects. */
  void setupPublishers();

  /** Starts up the UDP server. */
  bool startUDPServer();

  /** Send a control input message from CRS to the car. */
  void sendControlInput(const crs_msgs::car_input::ConstPtr& msg);

  /** Publish the received IMU data on /ns/imu. */
  void publishImuData(const IMUMeasurement& data);

  /** Publish the car's low-level motor inputs on /ns/car_ll_control_input. */
  void publishLowLevelControlInput(const SingleControlInput& reference, const MotorInput& drive_input,
                                   const MotorInput& steer_input);

  /** Publish the car's steering position on /ns/car_steer_state */
  void publishSteerState(const SteeringPositionMeasurement& steer_state);

  /** Publish the car's raw received Lighthouse frames on /ns/lighthouse_raw */
  void publishLighthouseData(const LighthouseFrame& lighthouse_frame);

  /** Publish the car's received Lighthouse sweeps on /ns/lighthouse_sweep */
  void publishLighthouseSweep(const LighthouseSweep& lighthouse_sweep, bool first_sweep);

  /** Determines if recived sweep is first or second sweep and publishes it */
  void handleLighthouseSweep(const LighthouseSweep& lighthouse_sweep);
  /* Private member variables ----------------------------------------------- */

  /** Retains the node handle passed to the constructor. */
  ros::NodeHandle& node_handle_;

  /** Subscriber object to control_input topic */
  ros::Subscriber sub_control_input_;

  /** Publisher on the car_ll_control_input topic */
  ros::Publisher pub_car_ll_control_input_;

  /** Publisher on the imu topic */
  ros::Publisher pub_imu_;

  /** Publisher on the car_steer_state topic */
  ros::Publisher pub_car_steer_state_;

  /** Publisher on the battery_state topic */
  ros::Publisher pub_battery_;
  /** Publisher on the wheel_speed topic */
  ros::Publisher pub_wheel_speed_;

  /** Publisher on the lighthouse raw frame topic */
  ros::Publisher pub_lighthouse_frame_;

  /** Publisher on the lighthouse sweep topic */
  ros::Publisher pub_lighthouse_sweep_;

  /**
   * UDP server object that handles the sending and receiving of UDP packets at
   * the byte level. Does not know about the protocol buffer layer.
   */
  UDPServer udp_server_;

  /** Port number that the server listens on. */
  int udp_port_;

  /** UDP socket that the node opened */
  int sock_ = -1;

  /** Open a UDP socket according to the configuration. */
  bool openSocket();

  /** Address of the car connecting to the node. */
  struct sockaddr_storage client_;
  socklen_t client_len_;

  /** The last recived first lighthouse sweep per basestation */
  LighthouseSweep last_first_lighthouse_sweep[LIGHTHOUSE_BASE_STATIONS];
  /** Indicates if the last lighthouse sweep was already published or still needs to be published, per basestation*/
  bool last_lighthouse_sweep_published[LIGHTHOUSE_BASE_STATIONS];

};
#endif
