cmake_minimum_required(VERSION 2.8.3)
project(crs_launch)


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

###########
# Tests #
##########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  # Controller Tests
    add_rostest_gtest(pid_controller_test 
    test/closed_loop_controller_tests/pid_controller_test.launch 
    test/closed_loop_controller_tests/controller_test.cpp)
    target_link_libraries(pid_controller_test ${catkin_LIBRARIES}  casadi stdc++fs)


    add_rostest_gtest(ff_fb_controller_test 
    test/closed_loop_controller_tests/ff_fb_controller_test.launch 
    test/closed_loop_controller_tests/controller_test.cpp)
    target_link_libraries(ff_fb_controller_test ${catkin_LIBRARIES}  casadi stdc++fs)


    add_rostest_gtest(mpcc_controller_test 
    test/closed_loop_controller_tests/mpcc_controller_test.launch 
    test/closed_loop_controller_tests/controller_test.cpp)
    target_link_libraries(mpcc_controller_test ${catkin_LIBRARIES}  casadi stdc++fs)


    add_rostest_gtest(mpcc_with_estimator_test 
    test/closed_loop_controller_tests/mpcc_with_estimator_test.launch 
    test/closed_loop_controller_tests/controller_test.cpp)
    target_link_libraries(mpcc_with_estimator_test ${catkin_LIBRARIES}  casadi stdc++fs)


  # Estimator Tests
    # Lowpass 
      add_rostest_gtest(pacejka_lowpass_meas_noise 
        test/closed_loop_estimator_tests/lowpass/pacejka_lowpass_meas_noise.launch 
        test/closed_loop_estimator_tests/estimator_test.cpp)
      target_link_libraries(pacejka_lowpass_meas_noise ${catkin_LIBRARIES}  casadi stdc++fs)

      add_rostest_gtest(pacejka_lowpass_no_meas_noise 
        test/closed_loop_estimator_tests/lowpass/pacejka_lowpass_no_meas_noise.launch 
        test/closed_loop_estimator_tests/estimator_test.cpp)
      target_link_libraries(pacejka_lowpass_no_meas_noise ${catkin_LIBRARIES}  casadi stdc++fs)

    # EKF
      # Pacejka
        add_rostest_gtest(pacejka_ekf_test_no_meas_noise 
        test/closed_loop_estimator_tests/ekf/pacejka_ekf_test_no_meas_noise.launch 
        test/closed_loop_estimator_tests/estimator_test.cpp)
        target_link_libraries(pacejka_ekf_test_no_meas_noise ${catkin_LIBRARIES}  casadi stdc++fs)

        add_rostest_gtest(pacejka_ekf_test_with_meas_noise 
        test/closed_loop_estimator_tests/ekf/pacejka_ekf_test_with_meas_noise.launch 
        test/closed_loop_estimator_tests/estimator_test.cpp)
        target_link_libraries(pacejka_ekf_test_with_meas_noise ${catkin_LIBRARIES}  casadi stdc++fs)

      # Kinematic
        add_rostest_gtest(kinematic_ekf_test_with_meas_noise 
        test/closed_loop_estimator_tests/ekf/kinematic_ekf_test_with_meas_noise.launch 
        test/closed_loop_estimator_tests/estimator_test.cpp)
        target_link_libraries(kinematic_ekf_test_with_meas_noise ${catkin_LIBRARIES}  casadi stdc++fs)

endif()



###############
# Executables #
###############


##########
# Export #
##########

cs_install()


