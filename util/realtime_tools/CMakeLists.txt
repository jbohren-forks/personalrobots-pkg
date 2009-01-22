cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(realtime_tools)
rospack_add_library(realtime_tools src/realtime_tools.cpp)
