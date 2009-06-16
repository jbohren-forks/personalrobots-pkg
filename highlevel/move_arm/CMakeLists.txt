cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(move_arm)

rospack_add_executable(move_arm src/move_arm.cpp)
rospack_add_executable(move_arm_action_test test/test_move_arm.cpp)


