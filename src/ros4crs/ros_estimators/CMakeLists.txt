cmake_minimum_required(VERSION 2.8.3)
project(ros_estimators)

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
  list(APPEND CPP_SRC_FILES src/car_estimator/pacejka_estimator.cpp)
else()
  message("Pacejka Model not found. Building without Pacejka model support")
endif()

if (${kinematic_model_FOUND})
  add_definitions(-Dkinematic_model_FOUND)
  list(APPEND CPP_SRC_FILES src/car_estimator/kinematic_estimator.cpp)
else()
  message("Kinematic Model not found. Building without kinematic model support")
endif()



if (${lowpass_estimator_FOUND})
  add_definitions(-Dlowpass_estimator_FOUND)
else()
  message("Lowpass Estimator not found. Building without Lowpass Estimator support")
endif()

if (${kalman_estimator_FOUND})
  add_definitions(-Dkalman_estimator_FOUND)
else()
  message("Kalman Estimator not found. Building without Kalman Estimator support")
endif()

cs_add_library(${PROJECT_NAME}
        ${CPP_SRC_FILES}
        src/data_converter.cpp
        src/component_registry.cpp
        )

target_link_libraries(${PROJECT_NAME} casadi)


###############
# Executables #
###############
cs_add_executable(ros_estimation_node
  app/ros_estimation_node.cpp
        )
target_link_libraries(ros_estimation_node ${PROJECT_NAME} casadi stdc++fs)

##########
# Tests #
##########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  if (${kalman_estimator_FOUND})
    add_rostest_gtest(test_node test/pacejka_kalman_test.launch test/pacejka_kalman_test.cpp)
    target_link_libraries(test_node ${catkin_LIBRARIES}  ${PROJECT_NAME} casadi stdc++fs)
  endif()

endif()


##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)