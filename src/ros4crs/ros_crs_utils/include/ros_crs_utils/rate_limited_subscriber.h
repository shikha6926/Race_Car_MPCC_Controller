#ifndef RATE_LIMITED_SUBSCRIBER_H
#define RATE_LIMITED_SUBSCRIBER_H

#include <ros/ros.h>
#include <memory>
#include <commons/static_track_trajectory.h>

#include <Eigen/Core>

namespace ros_utils
{
template <typename MessageType>
class RateLimitedSubscriber
{
private:
  MessageType last_message_;
  ros::Subscriber subscription_;
  ros::NodeHandlePtr nh_;
  double rate_;

public:
  RateLimitedSubscriber(const std::string& topic, double rate, const ros::NodeHandlePtr nh) : rate_(rate), nh_(nh)
  {
  }
  template <class T>
  ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(MessageType), T* obj,
                            const TransportHints& transport_hints = TransportHints())
  {
    subscription_ =
        nh_->subscribe(topic, queue_size, &RateLimitedSubscriber::msgCachingCallback, this, transport_hints);

    boost::bind(fp, obj, boost::placeholders::_1)
  }

  void msgCachingCallback(MessageType msg)
  {
    last_message_ = msg;
  }
};
}  // namespace ros_utils
#endif /* RATE_LIMITED_SUBSCRIBER_H */
