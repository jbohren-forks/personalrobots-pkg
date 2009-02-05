cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
add_definitions(-Wall)
rospack(robot_kinematics)
rospack_add_library(robot_kinematics
 src/robot_kinematics.cpp
 src/kdl_chain_wrapper.cpp
 src/serial_chain.cpp)

rospack_add_executable(bin/robot_kinematics_test test/robot_kinematics_test.cpp)
target_link_libraries(bin/robot_kinematics_test robot_kinematics)

rospack_add_gtest_future(bin/test_rev_rne test/rev_rne.cpp)
target_link_libraries(bin/test_rev_rne robot_kinematics)

rospack_add_executable(bin/rne_speed_test test/rne_speed.cpp)
target_link_libraries(bin/rne_speed_test robot_kinematics)


