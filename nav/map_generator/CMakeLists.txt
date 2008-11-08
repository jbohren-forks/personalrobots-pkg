cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(map_generator)
rospack_add_executable(map_generator src/map_generator.cpp)
