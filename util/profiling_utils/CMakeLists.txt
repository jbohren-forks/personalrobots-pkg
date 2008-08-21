cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(profiling_utils)
rospack_add_library(profiling_utils src/profiler.cpp)
