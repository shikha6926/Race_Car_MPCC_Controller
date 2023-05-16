cmake_minimum_required(VERSION 2.8.3)
project(pacejka_mpc_safety_filter)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
find_package(casadi)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}

  $ENV{ACADOS_SOURCE_DIR}/include
  $ENV{ACADOS_SOURCE_DIR}/include/acados
  $ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include
  $ENV{ACADOS_SOURCE_DIR}/include/hpipm/include
)

#############
# Libraries #
#############
#

cs_add_library(${PROJECT_NAME}
src/pacejka_mpc_safety_filter.cpp
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