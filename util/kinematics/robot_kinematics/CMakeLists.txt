cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rospack(robot_kinematics)
rospack_add_library(robot_kinematics src/robot_kinematics.cpp src/serial_chain.cpp)

rospack_add_executable(bin/robot_kinematics_test test/robot_kinematics_test.cpp)
target_link_libraries(bin/robot_kinematics_test robot_kinematics)


