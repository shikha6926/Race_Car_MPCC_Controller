#ifndef SRC_ROS_ROS_CRS_UTILS_INCLUDE_ROS_CRS_UTILS_STATE_MESSAGE_CONVERSION
#define SRC_ROS_ROS_CRS_UTILS_INCLUDE_ROS_CRS_UTILS_STATE_MESSAGE_CONVERSION

#ifdef kinematic_model_FOUND
#include <kinematic_model/kinematic_params.h>
#include <kinematic_model/kinematic_car_input.h>
#include <kinematic_model/kinematic_car_state.h>
#endif

namespace message_conversion
{

#ifdef kinematic_model_FOUND
struct kinematic_information
{
  crs_models::kinematic_model::kinematic_car_input input;
  crs_models::kinematic_model::kinematic_params params;
};
#endif

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

/**
 * @brief Convert a ros message to a state struct
 *
 * @tparam T the [T]arget state struct
 * @tparam S the [S]ource ros message
 * @param state
 * @return T
 */
template <typename T, typename S>
T convertMsgToState(const S& state);

/**
 * @brief Converts an internal crs input to a ros input
 *
 * @tparam RosInput ros message
 * @tparam InputType intenral crs input
 * @param input
 * @return RosInput
 */
template <typename RosInput, typename InputType>
RosInput convertToRosInput(const InputType input);

template <typename RosInput, typename CrsInput>
CrsInput convertToCrsInput(const RosInput input);

}  // namespace message_conversion
#endif /* SRC_ROS_ROS_CRS_UTILS_INCLUDE_ROS_CRS_UTILS_STATE_MESSAGE_CONVERSION */
