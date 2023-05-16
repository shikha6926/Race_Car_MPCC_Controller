cmake_minimum_required(VERSION 2.8.3)
project(mpc_controller)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
find_package(casadi REQUIRED)

catkin_simple()
catkin_package()

if(${acados_pacejka_mpcc_solver_FOUND})
   add_definitions(-Dacados_pacejka_mpcc_solver_FOUND)
else()
   message("Acados Pacejka MPCC Solver not found. Building without AcadosMPCC support")
endif()

if(${acados_pacejka_tracking_mpc_solver_FOUND})
   add_definitions(-Dacados_pacejka_tracking_mpc_solver_FOUND)
else()
   message("Acados Pacejka tracking MPC Solver not found. Building without AcadosTrackingMPC support")
endif()

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

cs_add_library(${PROJECT_NAME}
  src/pacejka_controller/mpcc_pacejka_controller.cpp
  src/pacejka_controller/tracking_mpc_pacejka_controller.cpp)
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
