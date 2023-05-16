cmake_minimum_required(VERSION 2.8.3)
project(dynamic_models)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
find_package(casadi)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
# Libraries #
#############
#


##########
# Tests #
##########


##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)