cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wg_robot_models)
rospack_add_library(RobotModels src/robot_models/kinematic.cpp)
