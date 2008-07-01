cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(bumblebee_bridge)
rospack_add_executable(bumblebee_bridge bumblebee_bridge.cpp)
