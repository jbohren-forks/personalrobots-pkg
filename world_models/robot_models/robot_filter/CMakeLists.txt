cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(robot_filter)
rospack_add_library(robot_filter src/robot_filter.cpp)
