/**
 * @file WiFiCom.cpp
 * @author Lukas Vogel (vogellu@ethz.ch)
 * @brief Class that communicates with cars over Wi-Fi and UDP.
 */

#include <arpa/inet.h>
#include <boost/interprocess/streams/bufferstream.hpp>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <string>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "Packet.pb.h"
#include "crs_msgs/car_input.h"
#include "crs_msgs/car_ll_control_input.h"
#include "crs_msgs/car_steer_state.h"
#include "crs_msgs/car_wheel_speed.h"
#include "crs_msgs/lighthouse_frame.h"
#include "crs_msgs/lighthouse_sweep.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "wifi_com/WiFiCom.h"

/** Default port number to open the UDP server on in case nothing was specified
    in the config file. */
#define DEFAULT_PORT_NUM 20211

/** Maximum recommended size of a UDP packet that is sent without complaining. */
#define UDP_MAX_RECOMMENDED_SIZE 256
/** Gravitational acceleration, used to convert IMU data from g's to m/s^2 */
#define GRAVITATIONAL_ACCELERATION 9.81

/* Static helper functions -------------------------------------------------- */

// The following functions are declared object-file static because including them in the
// header file would mean all the Protobuf declared types need to be visible to other
// files including the WiFiCom.h file.

/** Publish the wheel speed data to the namespace of this node. */
static void publishWheelSpeedData(const ros::Publisher& publisher,
                                  const WheelSpeedMeasurement& wheel_speed_measurement);
/** Publish the battery data to the namespace of this node. */
static void publishBatteryState(const ros::Publisher& publisher, const BatteryState& battery_state);

/* Public method implementation --------------------------------------------- */

WiFiCom::WiFiCom(ros::NodeHandle& n) : node_handle_(n), udp_port_(DEFAULT_PORT_NUM)
{
  loadParameters();
  setupSubscribers();
  setupPublishers();
  startUDPServer();

  for (size_t i = 0; i < LIGHTHOUSE_BASE_STATIONS; i++)
  {
    last_first_lighthouse_sweep[i] = LighthouseSweep();
    last_lighthouse_sweep_published[i] = true;
  }
  
  return;
}

WiFiCom::~WiFiCom()
{
  return;
}

void WiFiCom::poll()
{
  uint8_t rx_buffer[512];
  ssize_t recv_len = udp_server_.pollReceive(&client_, rx_buffer, sizeof(rx_buffer));

  if (recv_len < 0)
  {
    ROS_ERROR_STREAM("Polling of UDP server threw an error!" << recv_len);
    return;
  }
  if (recv_len == 0)
  {
    // no new data arrived
    return;
  }

  // Byte array is null terminated, so can cast to string
  boost::interprocess::bufferstream rx_istream((char*)rx_buffer, recv_len);

  Packet p;
  if (!p.ParseFromIstream(&rx_istream))
  {
    ROS_ERROR("Could not parse packet from input stream.");
  }

  if (p.has_car_state())
  {
    const CarState& state = p.car_state();
    if (state.has_drive_motor_input() && state.has_steer_motor_input() && state.has_current_reference())
    {
      publishLowLevelControlInput(state.current_reference(), state.drive_motor_input(), state.steer_motor_input());
    }

    if (state.has_imu_data())
    {
      publishImuData(state.imu_data());
    }

    if (state.has_steer_data())
    {
      publishSteerState(state.steer_data());
    }

    if (state.has_wheel_speed_data())
    {
      publishWheelSpeedData(pub_wheel_speed_, state.wheel_speed_data());
    }

    if (state.has_battery_state())
    {
      publishBatteryState(pub_battery_, state.battery_state());
    }

    for (int i = 0; i < state.lighthouse_data_size(); i++) {
      publishLighthouseData(state.lighthouse_data(i));
    }

    for (int i = 0; i < state.lighthouse_sweeps_size(); i++) {
      handleLighthouseSweep(state.lighthouse_sweeps(i));
    }
  }
  else if (p.has_ping())
  {
    ROS_INFO_STREAM_THROTTLE(60, "Connection to " << ros::this_node::getNamespace() << " is alive.");
  }
  else
  {
    ROS_WARN("Received unknown packet type, dropping...");
  }
}

void WiFiCom::controllerCallback(const crs_msgs::car_input::ConstPtr& msg)
{
  sendControlInput(msg);
}

/* Private method implementation -------------------------------------------- */

void WiFiCom::loadParameters()
{
  ROS_INFO("WiFiCom: loading parameters");
  if (!node_handle_.getParam("udp_port", udp_port_))
  {
    ROS_WARN_STREAM("No port number from config, using standard port: " << udp_port_);
  }
}

void WiFiCom::setupSubscribers()
{
  sub_control_input_ = node_handle_.subscribe("control_input", 1, &WiFiCom::controllerCallback, this);
}

void WiFiCom::setupPublishers()
{
  // While all these messages might arrive in the same packet from the car, each
  // belongs to a separate CRS topic and they are thus published on those:
  pub_car_ll_control_input_ = node_handle_.advertise<crs_msgs::car_ll_control_input>("car_ll_control_input", 1);
  pub_imu_ = node_handle_.advertise<sensor_msgs::Imu>("imu", 1);
  pub_car_steer_state_ = node_handle_.advertise<crs_msgs::car_steer_state>("car_steer_state", 1);
  pub_battery_ = node_handle_.advertise<sensor_msgs::BatteryState>("battery", 1);
  pub_wheel_speed_ = node_handle_.advertise<crs_msgs::car_wheel_speed>("wheel_speed", 1);
  pub_lighthouse_frame_ = node_handle_.advertise<crs_msgs::lighthouse_frame>("lighthouse_raw", 8);
  pub_lighthouse_sweep_ = node_handle_.advertise<crs_msgs::lighthouse_sweep>("lighthouse", 10);
}

bool WiFiCom::startUDPServer()
{
  udp_server_.setPortNum(udp_port_);
  if (!udp_server_.startListening())
  {
    ROS_ERROR("Could not start listening on UDP server!");
    return false;
  }
  ROS_INFO_STREAM("WiFiCom: Started listening on UDPServer, port = " << udp_port_);
  return true;
}

void WiFiCom::sendControlInput(const crs_msgs::car_input::ConstPtr& msg)
{
  Packet p;
  SingleControlInput* inp = p.mutable_control_input();
  inp->set_torque_ref(msg->torque);
  inp->set_steer_ref(msg->steer);

  // Handle case where the car's internal steering map should be overriden and
  // a raw potentiometer reference sent. This is indicated by the steer_override
  // flag in the car_input message.
  if (msg->steer_override)  // NOLINT
  {
    inp->mutable_steer_input()->set_steer_voltage(msg->steer);
  }
  else
  {
    inp->mutable_steer_input()->set_steer_angle(msg->steer);
  }

  std::string serialized_bytes;
  p.SerializeToString(&serialized_bytes);

  // Packets that are too long may be transferred in more than one transaction,
  // which is untested both in the server as in the client code.
  if (serialized_bytes.length() > UDP_MAX_RECOMMENDED_SIZE)
  {
    ROS_WARN_STREAM("Packet length is " << serialized_bytes.length() << "B, while recommended limit is "
                                        << UDP_MAX_RECOMMENDED_SIZE << "B!");
  }

  if (client_.ss_family == AF_INET || client_.ss_family == AF_INET6)
  {
    if (!udp_server_.send(&client_, (uint8_t*)serialized_bytes.c_str(), serialized_bytes.length()))
    {
      ROS_ERROR("Failed to send packet!");
    }
  }
}

void WiFiCom::publishImuData(const IMUMeasurement& data)
{
  sensor_msgs::Imu msg;

  // Convention: Set covariance of sensor measurement "orientation" to -1 if
  // this message field is invalid (which it is here, we don't have an absolute
  // orientation measurement yet). See ROS documentation for sensor_msgs/imu.
  msg.orientation_covariance[0] = -1;

  // Remap axes and adjust units:
  // - x and y axes need to be reversed
  msg.linear_acceleration.x = -data.linear_acceleration().x();
  msg.linear_acceleration.y = -data.linear_acceleration().y();
  msg.linear_acceleration.z = +data.linear_acceleration().z();
  msg.angular_velocity.x = -data.angular_velocity().x();
  msg.angular_velocity.y = -data.angular_velocity().y();
  msg.angular_velocity.z = +data.angular_velocity().z();

  msg.header.stamp = ros::Time::now();

  pub_imu_.publish(msg);
}

void WiFiCom::publishLowLevelControlInput(const SingleControlInput& reference, const MotorInput& drive_input,
                                          const MotorInput& steer_input)
{
  crs_msgs::car_ll_control_input msg;

  msg.drive_power = drive_input.power();
  msg.steer_power = steer_input.power();

  msg.steer_ref = reference.steer_ref();
  msg.torque_ref = reference.torque_ref();

  pub_car_ll_control_input_.publish(msg);
}

void WiFiCom::publishSteerState(const SteeringPositionMeasurement& steer_state)
{
  crs_msgs::car_steer_state msg;
  msg.steer_angle = steer_state.steer_rad();
  msg.steer_discrete_pos = steer_state.adc_meas();

  // This is not optimal yet, so it is not yet published.
  // Lower two bytes are the minimum position, upper two bytes are the maximum
  // position
  // msg.steer_min_pos = steer_state.adc_limits;
  // msg.steer_max_pos = steer_state.adc_limits;
  pub_car_steer_state_.publish(msg);
}

/* Static helper function implementation ---------------------------------------------------------------------------- */

static void publishWheelSpeedData(const ros::Publisher& publisher, const WheelSpeedMeasurement& wheel_speed_measurement)
{
  crs_msgs::car_wheel_speed msg;
  msg.header.stamp = ros::Time::now();

  msg.front_left = wheel_speed_measurement.front_left();
  msg.front_right = wheel_speed_measurement.front_right();
  msg.back_left = wheel_speed_measurement.back_left();
  msg.back_right = wheel_speed_measurement.back_right();

  publisher.publish(msg);
}

static void publishBatteryState(const ros::Publisher& publisher, const BatteryState& battery_state)
{
  sensor_msgs::BatteryState msg;

  msg.header.stamp = ros::Time::now();

  // Basic, usually known
  msg.voltage = battery_state.voltage();
  msg.current = battery_state.has_current() ? battery_state.current() : NAN;

  // Set other fields according to specification if unmeasured
  msg.temperature = NAN;
  msg.charge = NAN;
  msg.capacity = NAN;
  msg.percentage = NAN;
  msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

  msg.location = "unknown";
  msg.serial_number = "unknown";

  publisher.publish(msg);
}

void WiFiCom::publishLighthouseData(const LighthouseFrame& lighthouse_frame)
{
  crs_msgs::lighthouse_frame msg;
  msg.sensor_id = lighthouse_frame.sensor_id();
  msg.polynomial = lighthouse_frame.polynomial();
  msg.pulse_width = lighthouse_frame.pulse_width();
  msg.sync_offset = lighthouse_frame.sync_offset();
  msg.beam_word = lighthouse_frame.beam_word();
  msg.timestamp = lighthouse_frame.timestamp();

  msg.header.stamp = ros::Time::now();
  
  pub_lighthouse_frame_.publish(msg);
}

void WiFiCom::publishLighthouseSweep(const LighthouseSweep& lighthouse_sweep, bool first_sweep)
{
  crs_msgs::lighthouse_sweep msg;

  double angle_offset;
  if (first_sweep) 
  {
    angle_offset = - M_PI + M_PI / 3;
  }
  else
  {
    angle_offset = - M_PI - M_PI / 3;
  }

  msg.angle_0 = lighthouse_sweep.angle_0() + angle_offset;
  msg.angle_1 = lighthouse_sweep.angle_1() + angle_offset;
  msg.angle_2 = lighthouse_sweep.angle_2() + angle_offset;
  msg.angle_3 = lighthouse_sweep.angle_3() + angle_offset;
  msg.first_timestamp = lighthouse_sweep.first_timestamp();
  msg.polynomial = lighthouse_sweep.polynomial();
  msg.sync_timestamp = lighthouse_sweep.sync_timestamp();

  // Check if angle is inside field of view
  if (msg.angle_0 < -1.5 || msg.angle_0 > 1.5)
  {
    return;
  }
  
  msg.first_sweep = first_sweep;

  msg.header.stamp = ros::Time::now();
  
  pub_lighthouse_sweep_.publish(msg);
}

void WiFiCom::handleLighthouseSweep(const LighthouseSweep& lighthouse_sweep)
{
  size_t base_station = lighthouse_sweep.polynomial() >> 1;
  if (base_station >= LIGHTHOUSE_BASE_STATIONS)
  {
    // Invalid base station id
    return;
  }
  // Time difference between last and current sweeps sync event
  int32_t time_diff = lighthouse_sweep.sync_timestamp() - last_first_lighthouse_sweep[base_station].sync_timestamp();
  // Check if abs(time_diff) < 100 in 24 bit representation 
  if (((time_diff + 100) & 0x00FFFFFF) < 200)
  {
    // Last sweep was first this is second
    if (!last_lighthouse_sweep_published[base_station])
    {
      publishLighthouseSweep(last_first_lighthouse_sweep[base_station], true);
      last_lighthouse_sweep_published[base_station] = true;
    }
    publishLighthouseSweep(lighthouse_sweep, false);
  } else {
    // Curent sweep dose not belong to last sweep
    // Check if last_first_lighthouse_sweep angle is within ~6 deg of current sweep angle
    if (abs(last_first_lighthouse_sweep[base_station].angle_0() - lighthouse_sweep.angle_0()) < 0.1)
    {
      // Current sweep is probably first sweep
      publishLighthouseSweep(lighthouse_sweep, true);
      last_first_lighthouse_sweep[base_station] = LighthouseSweep(lighthouse_sweep);
      last_lighthouse_sweep_published[base_station] = true;
    } else {
      // Some sweeps were dropped, reset
      last_first_lighthouse_sweep[base_station] = LighthouseSweep(lighthouse_sweep);
      last_lighthouse_sweep_published[base_station] = false;
    }
  }
}
