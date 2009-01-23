cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(kinematic_planning)

genmsg()

rospack_add_executable(kinematic_planning src/kinematic_planning.cpp)
rospack_add_executable(motion_validator src/motion_validator.cpp)
rospack_add_executable(state_validity_monitor src/state_validity_monitor.cpp)

add_subdirectory(test)
