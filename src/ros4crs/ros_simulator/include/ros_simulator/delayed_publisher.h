#ifndef ROS_SIMULATOR_DELAYED_PUBLISHER_H
#define ROS_SIMULATOR_DELAYED_PUBLISHER_H

#include <mutex>
#include <queue>
#include <ros/ros.h>

namespace ros_simulator
{

/**
 * @brief Class that wraps around a ros publisher and delays the publishing by a certain amount
 *
 */
class DelayedPublisher
{
private:
  // Node handles.
  ros::NodeHandle nh_;
  // Publisher
  ros::Publisher publisher_;
  //
  std::vector<ros::Timer> timers;
  // Publisher delay
  double delay_;
  // free timer indices
  std::deque<int> free_indices;
  // Mutex for thread safety
  std::unique_ptr<std::mutex> m;

  /**
   * @brief Real publish callback. Marks this thread as done by adding the position to free_indeices
   *
   */
  template <typename M>
  void delayedTimerCallback(ros::TimerEvent event, M& message, int position)
  {
    publisher_.publish(message);
    free_indices.push_back(position);
  }

public:
  DelayedPublisher(ros::NodeHandle nh, ros::Publisher publisher, double delay)
    : nh_(nh), publisher_(publisher), delay_(delay), free_indices(), timers()
  {
    m = std::make_unique<std::mutex>();
  };

  /**
   * @brief Delay publishing function. Starts a oneshot, ros timer that delays the publication of this message for a
   * fixed delay
   *
   * @param message message to publish
   */
  template <typename M>
  void publish(const M& message)
  {
    std::unique_lock<std::mutex> lock(*m);
    if (delay_ == 0.0)
    {
      publisher_.publish(message);
      return;
    }

    bool created_timer = false;
    if (!free_indices.empty())
    {
      int position = free_indices.front();
      free_indices.pop_front();
      timers[position] = nh_.createTimer(
          ros::Duration(delay_), boost::bind(&DelayedPublisher::delayedTimerCallback<M>, this, _1, message, position),
          true, true);
    }
    else
    {
      timers.push_back(nh_.createTimer(
          ros::Duration(delay_),
          boost::bind(&DelayedPublisher::delayedTimerCallback<M>, this, _1, message, timers.size()), true, true));
    }
  }
};
}  // namespace ros_simulator

#endif  // ROS_SIMULATOR_DELAYED_PUBLISHER_H
