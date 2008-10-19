cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
include(rosbuild)
rospack(slam_gmapping)
rospack_add_executable(slam_gmapping slam_gmapping.cpp main.cpp)
