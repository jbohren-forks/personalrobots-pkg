cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(robot_mechanism_model)
rospack_add_library(mechanism_model src/SimpleTransmission.cpp)
