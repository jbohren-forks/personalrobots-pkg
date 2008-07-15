cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(mechanism_control)

rospack_add_library(mechanism_control src/base_control.cpp)

