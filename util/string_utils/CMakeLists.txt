cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(string_utils)
rospack_add_library(string_utils src/string_utils.cpp)
