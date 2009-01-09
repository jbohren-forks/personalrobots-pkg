cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(planning_models)
rospack_add_library(planning_models src/kinematic.cpp)
