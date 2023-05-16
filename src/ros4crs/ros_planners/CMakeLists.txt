cmake_minimum_required(VERSION 2.8.3)
project(ros_planners)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)

catkin_simple()

catkin_package()

## Optional dependency handling

if(${pacejka_model_FOUND})
   add_definitions(-Dpacejka_model_FOUND)
else()
  message("Pacejka Model not found. Building without Pacejka model support")
endif()

#############
# Libraries #
#############

include_directories(
  include
  thirdparty
  ${catkin_INCLUDE_DIRS}
  )

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/component_registry.cpp
        )

target_link_libraries(${PROJECT_NAME} ${OPTIONAL_LIBS})

###########
# Tests #
##########


###############
# Executables #
###############
cs_add_executable(planner_node
  app/planner_node.cpp
        )

target_link_libraries(planner_node ${PROJECT_NAME})

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
