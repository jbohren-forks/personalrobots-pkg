cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(profiling_utils)
rospack_add_library(profiling_utils src/profiler.cpp)
