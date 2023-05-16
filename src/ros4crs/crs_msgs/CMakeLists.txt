cmake_minimum_required(VERSION 2.8.3)
project(crs_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg/car/core_messages
  FILES
  car_com.msg
  car_state_cart.msg
  car_input.msg
  car_steer_state.msg
  car_wheel_speed.msg
)

add_message_files(
  DIRECTORY msg/car/additional_messages
  FILES
  car_ll_control_input.msg
  lighthouse_frame.msg
  lighthouse_sweep.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
