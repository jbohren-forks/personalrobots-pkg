cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(misc_utils)

add_definitions(-Wall)

rospack_add_gtest(ring_buffer_test test/test_ring_buffer.cpp)

