cmake_minimum_required(VERSION 2.8.3)
project(ros_tf_republisher)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


catkin_package()

#############
# Libraries #
#############

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

#############
# Libraries #
#############

# cs_add_library(${PROJECT_NAME}
#         src/republisher.cpp
# )

# target_link_libraries(${PROJECT_NAME})

###########
# Tests #
##########


###############
# Executables #
###############

cs_add_executable(republisher_node
  app/republisher_node.cpp)
#target_link_libraries(republisher_node ${PROJECT_NAME})


##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)