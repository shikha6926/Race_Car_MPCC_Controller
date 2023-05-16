cmake_minimum_required(VERSION 2.8.3)
project(ros_battery_monitor)

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

###########
# Tests #
##########


###############
# Executables #
###############

cs_add_executable(battery_monitor_node
  app/battery_monitor_node.cpp)

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
