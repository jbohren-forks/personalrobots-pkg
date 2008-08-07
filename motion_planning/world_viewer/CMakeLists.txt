cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(planning_world_viewer)
rospack_add_executable(planning_world_viewer src/planning_world_viewer.cpp)
