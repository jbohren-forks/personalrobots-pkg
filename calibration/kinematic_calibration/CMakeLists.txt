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

rospack_add_boost_directories()

rospack_add_executable(arm_phasespace_grabber src/arm_phasespace_grabber.cpp)
target_link_libraries(arm_phasespace_grabber kinematic_calibration)
rospack_link_boost(arm_phasespace_grabber thread)

#rospack_add_executable(sensor_kinematics_grabber src/sensor_kinematics_grabber.cpp)
#target_link_libraries(sensor_kinematics_grabber kinematic_calibration)
#rospack_link_boost(sensor_kinematics_grabber thread)

rospack_add_executable(run_chain_dumper src/run_chain_dumper.cpp)
target_link_libraries(run_chain_dumper kinematic_calibration)
rospack_link_boost(run_chain_dumper thread)

rospack_add_executable(add_calibration_data src/add_calibration_data.cpp)
target_link_libraries(add_calibration_data kinematic_calibration)
rospack_link_boost(add_calibration_data thread)

rospack_add_executable(led_tracker src/led_tracker.cpp)
target_link_libraries(led_tracker kinematic_calibration)
rospack_link_boost(led_tracker thread)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(test EXCLUDE_FROM_ALL)
