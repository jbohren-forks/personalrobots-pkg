cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(dorylus)
add_definitions(-Wall)
rospack_add_executable(dorylus_node src/dorylus_node.cpp src/dorylus.cpp)

