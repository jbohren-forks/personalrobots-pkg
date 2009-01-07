cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(planning_models)
rospack_add_library(planning_models src/kinematic.cpp)
