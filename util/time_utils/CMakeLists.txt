cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(time_utils)
rospack_add_library(time_utils src/time_utils.cpp)
