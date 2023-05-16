cmake_minimum_required(VERSION 2.8.3)
project(lloyd_planner)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

include_directories(
  include
  thirdparty
  ${catkin_INCLUDE_DIRS}
)

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
    src/multi_pacejka_lloyd_planner.cpp
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
