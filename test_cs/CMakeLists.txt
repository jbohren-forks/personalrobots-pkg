cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(test_cs)

rospack_add_executable(test_cs src/test_cs.cpp)
