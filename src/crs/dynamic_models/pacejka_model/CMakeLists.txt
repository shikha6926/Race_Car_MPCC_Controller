cmake_minimum_required(VERSION 2.8.3)
project(pacejka_model)

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
        src/pacejka_continuous.cpp
        src/pacejka_discrete.cpp
        src/utils/data_conversion.cpp
        )

target_link_libraries(${PROJECT_NAME})

##########
# Tests #
##########

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(struct_tests test/struct_tests.cpp ) 
  target_link_libraries(struct_tests ${catkin_LIBRARIES} ${PROJECT_NAME} casadi)
  catkin_add_gtest(model_tests test/model_tests.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/)
  target_link_libraries(model_tests ${catkin_LIBRARIES} ${PROJECT_NAME} casadi stdc++fs)
endif()

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)