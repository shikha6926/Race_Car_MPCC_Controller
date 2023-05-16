cmake_minimum_required(VERSION 2.8.3)
project(ros_crs_utils)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)

catkin_simple()
catkin_package()


#############
# Libraries #
#############
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

list(APPEND CPP_SRC_FILES src/parameter_io.cpp src/trajectory_message_conversion.cpp)

if (${lowpass_estimator_FOUND})
  add_definitions(-Dlowpass_estimator_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_lowpass.cpp)
else()
  message("Lowpass Estimator not found. Building without Lowpass Estimator support")
endif()

if (${pid_controller_FOUND})
  add_definitions(-Dpid_controller_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_pid.cpp)
else()
  message("PID Controller not found. Building without PID controller support")
endif()

if (${ff_fb_controller_FOUND})
  add_definitions(-Dff_fb_controller_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_ff_fb.cpp)
else()
  message("FF FB Controller not found. Building without FF FB controller support")
endif()

if (${mpc_controller_FOUND})
  add_definitions(-Dmpc_controller_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_mpc.cpp)
else()
  message("MPC Controller not found. Building without mpc controller support")
endif()

if (${pacejka_model_FOUND})
  add_definitions(-Dpacejka_model_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_pacejka.cpp)
  list(APPEND CPP_SRC_FILES src/state_message_conversion_pacejka.cpp)
else()
  message("Pacejka Model not found. Building without Pacejka model support")
endif()

if (${kinematic_model_FOUND})
  add_definitions(-Dkinematic_model_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_kinematic.cpp)
  list(APPEND CPP_SRC_FILES src/state_message_conversion_kinematic.cpp)
else()
  message("Kinematic Model not found. Building without kinematic model support")
endif()

if (${pacejka_mpc_safety_filter_FOUND})
  add_definitions(-Dpacejka_mpc_safety_filter_FOUND)
  list(APPEND CPP_SRC_FILES src/parameter_io_pacejka_mpc_safety.cpp)
else()
  message("Pacejka MPC Safety not found. Building without Pacejka MPC Safety support")
endif()


cs_add_library(${PROJECT_NAME} ${CPP_SRC_FILES})

target_link_libraries(${PROJECT_NAME})


##########
# Tests #
##########


###############
# Executables #
###############

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)