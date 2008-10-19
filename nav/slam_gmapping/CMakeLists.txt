cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
include(rosbuild)
rospack(slam_gmapping)
rospack_add_executable(slam_gmapping src/slam_gmapping.cpp src/main.cpp)
