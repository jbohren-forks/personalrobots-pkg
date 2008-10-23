cmake_minimum_required(VERSION 2.6)

include(rosbuild)
rospack(joy_node)
rospack_add_executable(joy_node joy_node.cpp)

