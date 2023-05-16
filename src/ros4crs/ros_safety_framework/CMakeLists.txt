cmake_minimum_required(VERSION 2.8.3)
project(ros_safety_framework)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


find_package(casadi)
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
        src/component_registry.cpp
        )

target_link_libraries(${PROJECT_NAME} casadi)

###########
# Tests #
##########


###############
# Executables #
###############

cs_add_executable(safety_filter_node
  app/safety_filter_node.cpp)
target_link_libraries(safety_filter_node ${PROJECT_NAME})

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)