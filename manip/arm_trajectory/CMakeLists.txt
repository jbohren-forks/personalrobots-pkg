cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(arm_trajectory)
rospack_add_executable(arm_trajectory arm_trajectory.cpp)
