cmake_minimum_required(VERSION 2.8.3)
project(ros_controllers)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
find_package(casadi)
find_package(acados)

catkin_simple()

catkin_package()

## Optional dependency handling

if(${pacejka_model_FOUND})
   add_definitions(-Dpacejka_model_FOUND)
else()
  message("Pacejka Model not found. Building without Pacejka model support")
endif()

if(${pid_controller_FOUND})
   add_definitions(-Dpid_controller_FOUND)
else()
   message("PID Controller not found. Building without PID Controller support")
endif()

if(${mpc_controller_FOUND})
   add_definitions(-Dmpc_controller_FOUND)
else()
   message("MPC Controller not found. Building without MPC Controller support")
endif()

if(${ff_fb_controller_FOUND})
   add_definitions(-Dff_fb_controller_FOUND)
else()
   message("FF FB Controller not found. Building without FF FB Controller support")
endif()

#############
# Libraries #
#############

include_directories(
  include
  ${catkin_INCLUDE_DIRS}

  # Link against acados for MPC generation
  $ENV{ACADOS_SOURCE_DIR}/include
  $ENV{ACADOS_SOURCE_DIR}/include/acados
  $ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include
  $ENV{ACADOS_SOURCE_DIR}/include/hpipm/include
  )

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/dynamic_config.cpp
        src/component_registry.cpp
        src/trajectory/dynamic_trajectory_updater.cpp
        src/ros_joystick_controller.cpp
        )

target_link_libraries(${PROJECT_NAME} ${OPTIONAL_LIBS})

###########
# Tests #
##########


###############
# Executables #
###############
cs_add_executable(controller_node
  app/controller_node.cpp
        )

target_link_libraries(controller_node ${PROJECT_NAME} acados casadi)
# make sure configure headers are built before any node using them
add_dependencies(controller_node ${PROJECT_NAME}_gencfg)

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)