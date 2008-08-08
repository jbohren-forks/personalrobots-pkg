cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(gazebo_map_extruder)
rospack_add_executable(gazebo_map_extruder gazebo_map_extruder.cc)
