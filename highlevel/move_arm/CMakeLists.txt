cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(move_arm)

rospack_add_executable(move_arm src/move_arm.cpp)

rospack_add_executable(move_to_position test/move_to_position.cpp)
rospack_add_executable(oscillate_move test/oscillate_move.cpp)
