cmake_minimum_required(VERSION 2.8.3)
project(car_track_visualizer)


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
cs_add_library(${PROJECT_NAME}
        src/CarTrackVisualizer.cpp
        )

target_link_libraries(${PROJECT_NAME})

###########
# Tests #
##########


###############
# Executables #
###############

add_executable(car_track_visualizer_node 
  src/car_track_visualizer_node.cpp
)

target_link_libraries(car_track_visualizer_node ${PROJECT_NAME})
add_dependencies(car_track_visualizer_node crs_msgs_generate_messages_cpp)

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)

