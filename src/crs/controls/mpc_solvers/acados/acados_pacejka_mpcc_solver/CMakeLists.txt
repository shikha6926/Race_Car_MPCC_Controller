cmake_minimum_required(VERSION 2.8.3)
project(acados_pacejka_mpcc_solver)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src/c_generated_code
  $ENV{ACADOS_SOURCE_DIR}/include
  $ENV{ACADOS_SOURCE_DIR}/include/acados
  $ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include
  $ENV{ACADOS_SOURCE_DIR}/include/hpipm/include
)

#############
# Libraries #
#############

find_library(ACADOS_LIB 
  NAMES acados
        hpipm
        blasfeo
  PATHS $ENV{ACADOS_SOURCE_DIR}/lib
)

file(GLOB_RECURSE GENERATED_C_CODE RELATIVE ${CMAKE_SOURCE_DIR} "src/c_generated_code/*.c")
# Remove matlab dependency
list(FILTER GENERATED_C_CODE EXCLUDE REGEX ".*sfunction.*\\.c$")
list(FILTER GENERATED_C_CODE EXCLUDE REGEX ".*sim.*\\.c$")

cs_add_library(${PROJECT_NAME}
        src/acados_pacejka_mpcc_solver.cpp # Main solver file used to interface with MPC controller
        ${GENERATED_C_CODE} # Generated c code
        )
target_link_libraries(${PROJECT_NAME}
	${ACADOS_LIB})

##########
# Tests #
##########

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include LIBRARIES ${PROJECT_NAME} ${ACADOS_LIB})