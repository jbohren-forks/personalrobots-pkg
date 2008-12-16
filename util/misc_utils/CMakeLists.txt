cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(misc_utils)
add_definitions(-Wall)

rospack_add_gtest(ring_buffer_test test/test_ring_buffer.cpp)

