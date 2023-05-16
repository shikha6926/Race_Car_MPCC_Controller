cmake_minimum_required(VERSION 2.8.3)
project(pacejka_sensor_model)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
find_package(casadi)
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
        src/vicon_sensor_model.cpp
        src/imu_sensor_model.cpp
        src/lighthouse_sensor_model.cpp
        )

target_link_libraries(${PROJECT_NAME})

##########
# Tests #
##########

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)