cmake_minimum_required(VERSION 2.8.3)
project(ff_fb_controller)

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
  src/ff_fb_controller.cpp 
        )

target_link_libraries(${PROJECT_NAME})


##########
# Tests #
##########

# Currently there are only closed loop test (located at crs_launch) for this controller

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)