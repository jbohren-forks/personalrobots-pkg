cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(robot_filter)
rospack_add_library(robot_filter src/robot_filter.cpp)
