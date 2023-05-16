#ifndef ROS_ESTIMATORS_DATA_CONVERTER_H
#define ROS_ESTIMATORS_DATA_CONVERTER_H

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <crs_msgs/lighthouse_sweep.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_models/sensor_measurement.h>

namespace ros_estimators
{
/**
 * @brief Converts a TransformStamped into a measurement vector (x,y,yaw)
 *
 * @param msg the message from vicon
 * @param T_sensor the transformation to apply before extracting x,y, and yaw
 *                 (e.g. transform from track frame to shared global frame)
 * @return crs_sensor_models::measurement Measurement struct containing x,y, and yaw as well as a valid timestamp and
 * sensor key (vicon)
 *
 */
crs_sensor_models::measurement parseViconData2D(const geometry_msgs::TransformStamped::ConstPtr msg,
                                                const tf::StampedTransform& T_sensor);

/**
 * @brief Converts a Imu measurement into a measurement vector (x_acc, y_acc, yaw_rate)
 *
 * @param msg the message from the imu
 * @return crs_sensor_models::measurement Measurement struct containing (x_acc, y_acc, yaw_rate) as well as a valid
 * timestamp and sensor key (imu)
 *
 */
crs_sensor_models::measurement parseImuData2D(const sensor_msgs::Imu::ConstPtr msg);

/**
 * @brief Converts a Lighthouse sweep into a measurement vector (angle_0, angle_1, angle_2, angle_3)
 *
 * @param msg the message from the lighthouse sweep
 * @return crs_sensor_models::measurement Measurement struct containing (angle_0, angle_1, angle_2, angle_3) as well as a valid
 * timestamp and sensor key (lighthouse_1/lighthouse_2)
 *
 */
crs_sensor_models::measurement parseLighthouseSweep(const crs_msgs::lighthouse_sweep::ConstPtr msg);

/**
 * @brief Class that converts vicon measurements into a simple 3D measurement vector (x,y,yaw) with unwrapped yaw angle
 *
 */
class ViconConverter
{
private:
  // Frame lookup (get car pose realtive to track not world)
  tf::TransformListener listener_;
  // Used for yaw unwrapping
  double last_yaw_ = 0;
  int loop_counter_ = 0;
  std::unique_ptr<tf::StampedTransform> T_track_world_;

public:
  // If true, always update track transform (this allows to move track while driving)
  bool update_track_transform = false;
  std::string world_frame = "world";
  std::string track_frame = "world";

  ViconConverter(){};
  /**
   * @brief Construct a new Vicon Converter object
   *
   * @param update_track_transform If true, always update the transformation between world_frame and track_frame.
   * Otherwise cache first transformation
   * @param world_frame Name of the shared frame in vicon (usually world)
   * @param track_frame Name of the frame of the track
   */
  ViconConverter(bool update_track_transform, std::string& world_frame, std::string& track_frame);

  /**
   * @brief Converts a Vicon Measurement into a sensor measurment (x,y and yaw). Also unwraps the yaw angle.
   *
   * @param msg the vicon measurement
   * @return crs_sensor_models::measurement
   */
  crs_sensor_models::measurement parseData2D(const geometry_msgs::TransformStamped::ConstPtr msg);
};

}  // namespace ros_estimators
#endif /* ROS_ESTIMATORS_DATA_CONVERTER_H */
