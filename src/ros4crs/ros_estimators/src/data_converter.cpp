
#include "ros_estimators/data_converter.h"
namespace ros_estimators
{
crs_sensor_models::measurement parseViconData2D(const geometry_msgs::TransformStamped::ConstPtr msg,
                                                const tf::StampedTransform& T_sensor)
{
  tf::StampedTransform T_world_car;
  tf::transformStampedMsgToTF(*msg, T_world_car);
  auto car_pose = (T_sensor)*T_world_car;

  tf::Matrix3x3 car_as_rot_mat;
  car_as_rot_mat.setRotation(car_pose.getRotation());

  // Extract yaw
  tf2Scalar yaw, pitch, roll;
  car_as_rot_mat.getRPY(roll, pitch, yaw);

  crs_sensor_models::measurement measurement;
  measurement.sensor_key = "vicon";
  measurement.measurement_data = Eigen::Vector3d::Zero();
  measurement.measurement_data(0) = car_pose.getOrigin().getX();
  measurement.measurement_data(1) = car_pose.getOrigin().getY();
  measurement.measurement_data(2) = yaw;

  measurement.timestamp = msg->header.stamp.toSec();
  if (measurement.timestamp == 0)  // No stamp provided in input msg
    measurement.timestamp = ros::Time::now().toSec();
  return measurement;
}

crs_sensor_models::measurement parseImuData2D(const sensor_msgs::Imu::ConstPtr msg)
{
  crs_sensor_models::measurement measurement;
  measurement.sensor_key = "imu";

  measurement.measurement_data = Eigen::Vector3d::Zero();
  measurement.measurement_data(0) = msg->linear_acceleration.x;
  measurement.measurement_data(1) = msg->linear_acceleration.y;
  measurement.measurement_data(2) = msg->angular_velocity.z;

  measurement.timestamp = msg->header.stamp.toSec();
  if (measurement.timestamp == 0)  // No stamp provided in input msg
    measurement.timestamp = ros::Time::now().toSec();
  return measurement;
}

crs_sensor_models::measurement parseLighthouseSweep(const crs_msgs::lighthouse_sweep::ConstPtr msg)
{
  crs_sensor_models::measurement measurement;
  int base_station_id = msg->polynomial >> 1;
  if (msg->first_sweep)
  {
    measurement.sensor_key = "lighthouse_" + std::to_string(base_station_id) + "_1";
  } else {
    measurement.sensor_key = "lighthouse_" + std::to_string(base_station_id) + "_2";
  }

  measurement.measurement_data = Eigen::Vector4d::Zero();
  measurement.measurement_data(0) = msg->angle_0;
  measurement.measurement_data(1) = msg->angle_1;
  measurement.measurement_data(2) = msg->angle_2;
  measurement.measurement_data(3) = msg->angle_3;

  measurement.timestamp = msg->header.stamp.toSec();
  if (measurement.timestamp == 0)  // No stamp provided in input msg
    measurement.timestamp = ros::Time::now().toSec();
  return measurement;
}

template <class T>
inline int sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

ViconConverter::ViconConverter(bool update_track_transform, std::string& world_frame, std::string& track_frame)
  : update_track_transform(update_track_transform), world_frame(world_frame), track_frame(track_frame){};

crs_sensor_models::measurement ViconConverter::parseData2D(const geometry_msgs::TransformStamped::ConstPtr msg)
{
  crs_sensor_models::measurement measurement;
  // No initial track transform found or we request to update the track transform every time
  if (!T_track_world_ || update_track_transform)
  {
    tf::StampedTransform transform;
    listener_.lookupTransform(track_frame, world_frame, ros::Time(0), transform);
    T_track_world_.reset(new tf::StampedTransform);
    *T_track_world_ = std::move(transform);
  }
  measurement = parseViconData2D(msg, *T_track_world_);

  double yaw = measurement.measurement_data(2);
  // ================== Unwrap yaw from vicon ==================
  double yaw_raw_diff = last_yaw_ - yaw;
  last_yaw_ = yaw;
  if (yaw_raw_diff >= M_PI)
    loop_counter_++;
  else if (yaw_raw_diff <= -M_PI)
    loop_counter_--;

  if (std::abs(loop_counter_) <= 1)
    measurement.measurement_data(2) = loop_counter_ * M_PI + (sign(loop_counter_) * M_PI + yaw);
  else
    measurement.measurement_data(2) = loop_counter_ * 2 * M_PI + yaw;
  return measurement;
}

}  // namespace ros_estimators