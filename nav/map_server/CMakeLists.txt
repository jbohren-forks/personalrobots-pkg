cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
include(rosbuild)
rospack(map_server)
rospack_add_executable(map_server map_server.cc)
