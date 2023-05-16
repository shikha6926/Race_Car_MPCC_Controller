cmake_minimum_required(VERSION 2.8.3)
project(ros_simulator)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
find_package(casadi)
catkin_simple()
catkin_package()

#############
# Libraries #
#############
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if (${pacejka_model_FOUND})
  add_definitions(-Dpacejka_model_FOUND)
  list(APPEND CPP_SRC_FILES src/ros_pacejka_simulator.cpp)
else()
  message("Pacejka Model not found. Building without kinematic model support")
endif()

if (${kinematic_model_FOUND})
  add_definitions(-Dkinematic_model_FOUND)
  list(APPEND CPP_SRC_FILES src/ros_kinematic_simulator.cpp)
else()
  message("Kinematic Model not found. Building without kinematic model support")
endif()


cs_add_library(${PROJECT_NAME} src/component_registry.cpp ${CPP_SRC_FILES})

target_link_libraries(${PROJECT_NAME})

###########
# Tests #
##########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  if (${pacejka_model_FOUND})
    add_rostest_gtest(test_simulator_node test/ros_simulator_test.launch test/ros_simulator_test.cpp)
    # catkin_add_gtest(my_unit_test test/MyClass_test.cpp)
    target_link_libraries(test_simulator_node ${catkin_LIBRARIES}  ${PROJECT_NAME} casadi stdc++fs)
  endif()
endif()


###############
# Executables #
###############
cs_add_executable(simulation_node
  app/simulation_node.cpp
        )
target_link_libraries(simulation_node ${PROJECT_NAME} casadi stdc++fs)

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)