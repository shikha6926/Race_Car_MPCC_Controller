cmake_minimum_required(VERSION 2.8.3)
project(commons)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#############
# Libraries #
#############


cs_add_library(${PROJECT_NAME}
        src/filter.cpp
        src/trajectory.cpp
        src/static_track_trajectory.cpp
        src/dynamic_point_trajectory.cpp
	  src/trajectory_spline.cpp
        )

target_link_libraries(${PROJECT_NAME})

##########
# Tests #
##########

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(track_tests test/track_tests.cpp)
  target_link_libraries(track_tests ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)