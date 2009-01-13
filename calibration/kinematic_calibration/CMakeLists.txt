cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(kinematic_calibration)
genmsg()
gensrv()
rospack_add_library(kinematic_calibration src/link_param_jacobian.cpp
                                          src/link_param_jacobian_solver.cpp
                                          src/jac_newmat_bridge.cpp
                                          src/chain_modifier.cpp
                                          src/parameter_estimator.cpp
                                          src/unittest_io.cpp
                                          src/unittest_verification.cpp
                                          )

rospack_add_executable(arm_phasespace_grabber src/arm_phasespace_grabber.cpp)
target_link_libraries(arm_phasespace_grabber kinematic_calibration)

rospack_add_executable(sensor_kinematics_grabber src/sensor_kinematics_grabber.cpp)
target_link_libraries(sensor_kinematics_grabber kinematic_calibration)

# ********** Tests **********
rospack_add_gtest(test/jacobian_unittest test/jacobian_unittest.cpp)
target_link_libraries(test/jacobian_unittest kinematic_calibration)

rospack_add_gtest(bridge_unittest test/bridge_unittest.cpp)
target_link_libraries(bridge_unittest kinematic_calibration)

rospack_add_gtest(test/chain_modifier_unittest test/chain_modifier_unittest.cpp)
target_link_libraries(test/chain_modifier_unittest kinematic_calibration)

rospack_add_gtest(error_vec_marker_3d_unittest test/error_vec_marker_3d_unittest.cpp)
target_link_libraries(error_vec_marker_3d_unittest kinematic_calibration)

rospack_add_gtest(estimator_marker_3d_unittest test/estimator_marker_3d_unittest.cpp)
target_link_libraries(estimator_marker_3d_unittest kinematic_calibration)
