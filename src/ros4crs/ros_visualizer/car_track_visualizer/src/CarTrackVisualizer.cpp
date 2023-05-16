#include "crs_msgs/car_state_cart.h"
#include "car_track_visualizer/CarTrackVisualizer.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmath>
#include <visualization_msgs/Marker.h>

#include <ros_crs_utils/parameter_io.h>

CarTrackVisualizer::CarTrackVisualizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  loadParameters();
  subscribeToState();
  setupPublisher();
  setupTrack();
  setupMarker();
}

void CarTrackVisualizer::loadParameters()
{
  ROS_INFO("Visualizer: loading visualizer parameters");
  // load node parameters
  if (!nh_private_.getParam("node_rate", node_rate_))
    ROS_WARN_STREAM("Visualizer: did not load visualizer node_rate.");

  // load node parameters
  if (!nh_private_.getParam("show_track_angle", show_track_angle_))
    ROS_WARN_STREAM("Visualizer: did not load visualizer show_track_angle.");
  // load node parameters
  if (!nh_private_.getParam("point_downsampling_factor", point_downsampling_factor_))
    ROS_WARN_STREAM("Visualizer: did not load visualizer point_downsampling_factor.");

  // load node parameters
  if (!nh_private_.getParam("lloyd_flag", lloyd_flag_))
    ROS_WARN_STREAM("Visualizer: did not load visualizer lloyd_flag.");

  nh_private_.getParam("trajectory/show_past_gt_trajectory", show_past_gt_trajectory_);
  nh_private_.getParam("trajectory/show_past_est_trajectory", show_past_est_trajectory_);
  nh_private_.getParam("trajectory/number_of_past_samples", number_of_past_samples_);

  nh_private_.getParam("estimate/color/r", COLOR_CAR_EST_[0]);
  nh_private_.getParam("estimate/color/g", COLOR_CAR_EST_[1]);
  nh_private_.getParam("estimate/color/b", COLOR_CAR_EST_[2]);
  nh_private_.getParam("estimate/color/a", COLOR_CAR_EST_[3]);

  nh_private_.getParam("gt/color/r", COLOR_CAR_GT_[0]);
  nh_private_.getParam("gt/color/g", COLOR_CAR_GT_[1]);
  nh_private_.getParam("gt/color/b", COLOR_CAR_GT_[2]);
  nh_private_.getParam("gt/color/a", COLOR_CAR_GT_[3]);
  nh_private_.getParam("publish_track", publish_track_);

  nh_private_.getParam("car_namespace", car_namespace_);

  static_track_trajectory_ = parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle("track"));
}

void CarTrackVisualizer::subscribeToState()
{
  sub_gt_ = nh_private_.subscribe<crs_msgs::car_state_cart>(
      "car_state_gt", 10, boost::bind(&CarTrackVisualizer::stateCallback, this, _1, false));
  sub_est_ = nh_private_.subscribe<crs_msgs::car_state_cart>(
      "car_state_est", 10, boost::bind(&CarTrackVisualizer::stateCallback, this, _1, true));
}

// float min_velocity_ = 1.5;
// float max_velocity_ = 2.5;
void CarTrackVisualizer::stateCallback(const boost::shared_ptr<crs_msgs::car_state_cart const> msg, bool is_estimation)
{
  auto& stamp = is_estimation ? last_est_callback_ : last_gt_callback_;
  if (msg->header.stamp.toSec() - stamp < 1 / getNodeRate())
    return;

  stamp = msg->header.stamp.toSec();

  if (is_estimation)
  {
    got_est_car_state_ = true;
    car_state_estimated_ = *msg;
  }
  else
  {
    got_car_state_ = true;
    car_state_ = *msg;
  }

  auto& trajectory_markers = is_estimation ? est_trajectory_ : gt_trajectory_;
  if ((show_past_est_trajectory_ && is_estimation) || (show_past_gt_trajectory_ && !is_estimation))
  {
    if (number_of_past_samples_ != -1 && trajectory_markers.points.size() > number_of_past_samples_)
    {
      trajectory_markers.points.erase(trajectory_markers.points.begin(),
                                      trajectory_markers.points.begin() +
                                          int(0.1 * number_of_past_samples_));  // remove first 10%
      trajectory_markers.colors.erase(trajectory_markers.colors.begin(),
                                      trajectory_markers.colors.begin() +
                                          int(0.1 * number_of_past_samples_));  // remove first 10%
    }
    geometry_msgs::Point p;
    p.x = msg->x;
    p.y = msg->y;
    p.z = 0;
    trajectory_markers.points.push_back(p);

    std_msgs::ColorRGBA color;
    double normalized_velocity =
        std::max(0.0, std::min(1.0, (msg->v_tot - min_velocity_) / (max_velocity_ - min_velocity_)));
    color.r = normalized_velocity < 0.5 ? normalized_velocity : 1 - normalized_velocity;
    color.g = normalized_velocity < 0.5 ? 0 : normalized_velocity;
    color.b = normalized_velocity < 0.5 ? 1 - normalized_velocity : 0;
    trajectory_markers.colors.push_back(color);
  }
}

void CarTrackVisualizer::setupPublisher()
{
  pub_ = nh_.advertise<visualization_msgs::Marker>("track_info", 200);
}

void CarTrackVisualizer::setupTrack()
{
  ROS_INFO("Visualizer: setting up Track...");
  track_.header.frame_id = FRAME_;
  track_.header.stamp = ros::Time::now();
  track_.ns = "track_center";
  track_.action = visualization_msgs::Marker::ADD;
  track_.pose.orientation.w = 1.0;
  track_.type = visualization_msgs::Marker::LINE_STRIP;
  track_.id = 1;
  track_.scale.x = TRACK_SCALE_;
  track_.scale.y = TRACK_SCALE_;
  track_.scale.z = TRACK_SCALE_;
  track_.color.r = ORANGE_[0];
  track_.color.g = ORANGE_[1];
  track_.color.b = ORANGE_[2];
  track_.color.a = ORANGE_[3];

  geometry_msgs::Point temp_p;
  int idx = 0;
  for (const auto& pt : static_track_trajectory_->getCenterLine())
  {
    if (idx++ % point_downsampling_factor_ != 0)
      continue;
    temp_p.x = pt.x();
    temp_p.y = pt.y();
    temp_p.z = -0.01;
    track_.points.push_back(temp_p);
  }

  boundary_.header.frame_id = FRAME_;
  boundary_.header.stamp = ros::Time::now();
  boundary_.ns = "track_boundary";
  boundary_.action = visualization_msgs::Marker::ADD;
  boundary_.pose.orientation.w = 1.0;
  boundary_.type = visualization_msgs::Marker::LINE_STRIP;
  boundary_.id = 2;
  boundary_.scale.x = TRACK_SCALE_;
  boundary_.scale.y = TRACK_SCALE_;
  boundary_.color.r = BLACK_[0];
  boundary_.color.g = BLACK_[1];
  boundary_.color.b = BLACK_[2];
  boundary_.color.a = BLACK_[3];

  const auto& center_line = static_track_trajectory_->getCenterLine();
  for (int i = 0; i < center_line.size(); i++)
  {
    if (i % point_downsampling_factor_ != 0)
      continue;
    temp_p.x =
        center_line[i].x() + static_track_trajectory_->getWidth() / 2.0 * static_track_trajectory_->getRate(i).y();
    temp_p.y =
        center_line[i].y() - static_track_trajectory_->getWidth() / 2.0 * static_track_trajectory_->getRate(i).x();
    temp_p.z = 0.0;
    boundary_.points.push_back(temp_p);
  }
  for (int i = 0; i < center_line.size(); i++)
  {
    if (i % point_downsampling_factor_ != 0)
      continue;
    temp_p.x =
        center_line[i].x() - static_track_trajectory_->getWidth() / 2.0 * static_track_trajectory_->getRate(i).y();
    temp_p.y =
        center_line[i].y() + static_track_trajectory_->getWidth() / 2.0 * static_track_trajectory_->getRate(i).x();
    boundary_.points.push_back(temp_p);
  }
}

void CarTrackVisualizer::setupMarker()
{
  uint32_t shape = visualization_msgs::Marker::CUBE;

  gt_car_marker_.header.frame_id = FRAME_;
  gt_car_marker_.header.stamp = ros::Time::now();
  gt_car_marker_.ns = car_namespace_ + "_groundtruth";
  gt_car_marker_.id = 0;

  // Set the marker type.
  gt_car_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  gt_car_marker_.mesh_resource = "package://car_track_visualizer/config/Basic_Beetle.stl";
  gt_car_marker_.action = visualization_msgs::Marker::ADD;
  gt_car_marker_.pose.position.x = 0;
  gt_car_marker_.pose.position.y = 0;
  gt_car_marker_.pose.position.z = 0;
  gt_car_marker_.pose.orientation.x = 0.0;
  gt_car_marker_.pose.orientation.y = 0.0;
  gt_car_marker_.pose.orientation.z = 0.0;
  gt_car_marker_.pose.orientation.w = 1.0;

  // Set the scale of the marker (side lenghts of cube)
  gt_car_marker_.scale.x = CAR_SCALE_;
  gt_car_marker_.scale.y = CAR_SCALE_;
  gt_car_marker_.scale.z = CAR_SCALE_;

  // Set the color -- be sure to set alpha to something non-zero!
  gt_car_marker_.color.r = COLOR_CAR_GT_[0];
  gt_car_marker_.color.g = COLOR_CAR_GT_[1];
  gt_car_marker_.color.b = COLOR_CAR_GT_[2];
  gt_car_marker_.color.a = COLOR_CAR_GT_[3];

  est_car_marker_.header.frame_id = FRAME_;
  est_car_marker_.header.stamp = ros::Time::now();
  est_car_marker_.ns = car_namespace_ + "_estimated";

  est_car_marker_.id = 0;

  // Set the marker type.
  est_car_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  est_car_marker_.mesh_resource = "package://car_track_visualizer/config/Basic_Beetle.stl";
  est_car_marker_.action = visualization_msgs::Marker::ADD;
  est_car_marker_.pose.position.x = 0;
  est_car_marker_.pose.position.y = 0;
  est_car_marker_.pose.position.z = 0;
  est_car_marker_.pose.orientation.x = 0.0;
  est_car_marker_.pose.orientation.y = 0.0;
  est_car_marker_.pose.orientation.z = 0.0;
  est_car_marker_.pose.orientation.w = 1.0;

  // Set the scale of the marker (side lenghts of cube)
  est_car_marker_.scale.x = CAR_SCALE_;
  est_car_marker_.scale.y = CAR_SCALE_;
  est_car_marker_.scale.z = CAR_SCALE_;

  // Set the color -- be sure to set alpha to something non-zero!
  est_car_marker_.color.r = COLOR_CAR_EST_[0];
  est_car_marker_.color.g = COLOR_CAR_EST_[1];
  est_car_marker_.color.b = COLOR_CAR_EST_[2];
  est_car_marker_.color.a = COLOR_CAR_EST_[3];

  // ===============================================
  //============== Trajectory history ==============
  // ===============================================

  est_trajectory_.header.frame_id = FRAME_;
  est_trajectory_.header.stamp = ros::Time::now();
  est_trajectory_.ns = car_namespace_ + "_estimated_trajectory";
  est_trajectory_.id = 0;

  // Set the trajectory marker type.
  est_trajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  est_trajectory_.action = visualization_msgs::Marker::ADD;
  est_trajectory_.pose.orientation.x = 0.0;
  est_trajectory_.pose.orientation.y = 0.0;
  est_trajectory_.pose.orientation.z = 0.0;
  est_trajectory_.pose.orientation.w = 1.0;

  // Set the scale of the marker (side lenghts of cube)
  est_trajectory_.scale.x = TRACK_SCALE_ * 0.5;
  est_trajectory_.scale.y = TRACK_SCALE_ * 0.5;
  est_trajectory_.scale.z = TRACK_SCALE_ * 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  est_trajectory_.color.r = 0;
  est_trajectory_.color.g = 1;
  est_trajectory_.color.b = 0;
  est_trajectory_.color.a = 1;

  gt_trajectory_.header.frame_id = FRAME_;
  gt_trajectory_.header.stamp = ros::Time::now();
  gt_trajectory_.ns = car_namespace_ + "_groundtruth_trajectory";
  gt_trajectory_.id = 0;

  // Set the trajectory marker type.
  gt_trajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  gt_trajectory_.action = visualization_msgs::Marker::ADD;
  gt_trajectory_.pose.orientation.x = 0.0;
  gt_trajectory_.pose.orientation.y = 0.0;
  gt_trajectory_.pose.orientation.z = 0.0;
  gt_trajectory_.pose.orientation.w = 1.0;

  // Set the scale of the marker (side lenghts of cube)
  gt_trajectory_.scale.x = TRACK_SCALE_ * 0.5;
  gt_trajectory_.scale.y = TRACK_SCALE_ * 0.5;
  gt_trajectory_.scale.z = TRACK_SCALE_ * 0.5;
}

void CarTrackVisualizer::updateMarker()
{
  // sets the quaternion entries based on the yaw angle of the car
  q_.setRPY(0, 0, car_state_.yaw + M_PI / 2.0);

  gt_car_marker_.pose.position.x = car_state_.x;
  gt_car_marker_.pose.position.y = car_state_.y;
  gt_car_marker_.pose.orientation.x = q_[0];
  gt_car_marker_.pose.orientation.y = q_[1];
  gt_car_marker_.pose.orientation.z = q_[2];
  gt_car_marker_.pose.orientation.w = q_[3];

  est_car_marker_.pose.position.x = car_state_estimated_.x;
  est_car_marker_.pose.position.y = car_state_estimated_.y;
  q_.setRPY(0, 0, car_state_estimated_.yaw + M_PI / 2.0);

  est_car_marker_.pose.orientation.x = q_[0];
  est_car_marker_.pose.orientation.y = q_[1];
  est_car_marker_.pose.orientation.z = q_[2];
  est_car_marker_.pose.orientation.w = q_[3];
}

float CarTrackVisualizer::getNodeRate()
{
  return node_rate_;
}

void CarTrackVisualizer::publishMarker()
{
  if (got_car_state_)
  {
    pub_.publish(gt_car_marker_);
  }
  if (got_est_car_state_)
  {
    pub_.publish(est_car_marker_);
  }

  if (show_past_gt_trajectory_)
  {
    pub_.publish(gt_trajectory_);
  }
  if (show_past_est_trajectory_)
  {
    pub_.publish(est_trajectory_);
  }

  // Only publish track if we have a new subscriber
  if (publish_track_ && (track_pub_counter_-- < 0 || track_subscribers_ != pub_.getNumSubscribers()))
  {
    track_pub_counter_ = publish_track_every_ith_iteration_;

    if(!lloyd_flag_)
    {
      if (show_track_angle_)
      {
        track_.type = visualization_msgs::Marker::ARROW;
        track_.action = visualization_msgs::Marker::ADD;
        track_.scale.y = TRACK_SCALE_;
        track_.scale.x = TRACK_SCALE_ * 3;

        track_.points.clear();
        track_.colors.clear();

        int idx = 0;
        track_.id = 0;
        for (const auto& pt : static_track_trajectory_->getCenterLine())
        {
          if (idx++ % (point_downsampling_factor_ * 3) != 0)
            continue;
            track_.id = track_.id + 1;
            double angle = static_track_trajectory_->getTrackAngle(idx);
            track_.pose.position.x = pt.x();
            track_.pose.position.y = pt.y();
            track_.pose.orientation.w = std::cos(angle * 0.5);
            track_.pose.orientation.x = 0;
            track_.pose.orientation.y = 0;
            track_.pose.orientation.z = std::sin(angle * 0.5);
            pub_.publish(track_);
          }
        }
        else
        {
          pub_.publish(track_);
        }
        pub_.publish(boundary_);
      }

    track_subscribers_ = pub_.getNumSubscribers();
  }
}

void CarTrackVisualizer::run()
{
  updateMarker();
  publishMarker();
}
