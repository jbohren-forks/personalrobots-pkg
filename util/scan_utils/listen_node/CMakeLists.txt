cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(listen_node)
add_definitions(-Wall)
rospack_add_library(listennode src/scanListenNode.cpp src/rosSystemCalls.cpp)

