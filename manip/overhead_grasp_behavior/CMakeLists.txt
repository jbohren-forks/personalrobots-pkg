cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(overhead_grasp_behavior)

rospack_add_executable(bin/overhead_grasp src/overhead_grasp.cc)


