cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(generic_controllers)
rospack_add_library(generic_controllers src/pid.cpp src/joint_position_controller.cpp)
