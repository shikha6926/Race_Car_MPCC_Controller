#ifndef SRC_ROS_ROS_CRS_UTILS_INCLUDE_ROS_CRS_UTILS_TRAJECTORY_MESSAGE_CONVERSION
#define SRC_ROS_ROS_CRS_UTILS_INCLUDE_ROS_CRS_UTILS_TRAJECTORY_MESSAGE_CONVERSION

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PolygonStamped.h>

namespace message_conversion
{

/**
 * @brief Converts a internal state struct to a ros message
 *
 * @tparam T the [T]arget ros message type
 * @tparam S the [S]ource state struct
 * @tparam I the [I]nput struct
 * @param state
 * @return T converted state
 */
template <typename T, typename S, typename I>
T convertStateToRosMsg(const S& state, const I& input);


template <typename TrajectoryType>
// TODO, we are really just hijakcing the joint trajectory format. There must either be a better suited message or we
// should define our own
trajectory_msgs::JointTrajectory convertToRosTrajectory(const std::vector<TrajectoryType>& trajectory,
                                                        std::string car_name = "");

geometry_msgs::PolygonStamped convertToRosVoronoi(const std::vector<double> vor_edges_x, const std::vector<double> vor_edges_y);

}  // namespace message_conversion
#endif /* SRC_ROS_ROS_CRS_UTILS_INCLUDE_ROS_CRS_UTILS_TRAJECTORY_MESSAGE_CONVERSION */
