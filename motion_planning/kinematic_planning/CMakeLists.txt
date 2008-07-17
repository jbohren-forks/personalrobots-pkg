cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(kinematic_planning)
rospack_add_executable(kinematic_planning src/kinematic_planning.cpp)
