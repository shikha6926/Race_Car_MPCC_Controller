cmake_minimum_required(VERSION 2.8.3)
project(lowpass_estimator)

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

cs_add_library(${PROJECT_NAME}
        src/pacejka_lowpass_estimator.cpp
)

##########
# Tests #
##########

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(lp_tests test/lowpass_test.cpp  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/)
  target_link_libraries(lp_tests ${PROJECT_NAME} ${catkin_LIBRARIES} casadi stdc++fs)
endif()

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)