cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(scan_utils)
add_definitions(-Wall)
rospack_add_library(scanutils src/smartScan.cpp src/dataTypes.cpp src/octree.cpp)
rospack_add_library(listennode src/listen_node/scanListenNode.cpp src/listen_node/rosSystemCalls.cpp)

