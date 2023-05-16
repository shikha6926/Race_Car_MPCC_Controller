cmake_minimum_required(VERSION 2.8.3)
project(ros_backtracker)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


catkin_package()

#############
# Libraries #
#############

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/crash_detector.cpp
        src/input_filter.cpp
        src/linear_backtracker.cpp
        )

target_link_libraries(${PROJECT_NAME})

###########
# Tests #
##########


###############
# Executables #
###############

cs_add_executable(crash_detector_node
  app/crash_detector_node.cpp)
target_link_libraries(crash_detector_node ${PROJECT_NAME})


cs_add_executable(input_filter_node
  app/input_filter_node.cpp)
target_link_libraries(input_filter_node ${PROJECT_NAME})


cs_add_executable(linear_backtracker_node
  app/linear_backtracker_node.cpp)
target_link_libraries(linear_backtracker_node ${PROJECT_NAME})


##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)