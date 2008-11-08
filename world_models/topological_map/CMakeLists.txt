cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(topological_map)
rospack_add_executable(bottleneck_graph src/bottleneck_graph.cpp)
