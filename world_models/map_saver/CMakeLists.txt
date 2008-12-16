cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(map_saver)
rospack_add_executable(map_saver src/map_saver.cpp)
