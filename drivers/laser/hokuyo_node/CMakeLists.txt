cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(hokuyo_node)
rospack_add_executable(hokuyo_node hokuyo_node.cpp)
