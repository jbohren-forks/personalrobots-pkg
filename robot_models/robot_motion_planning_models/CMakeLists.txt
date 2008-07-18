cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(robot_motion_planning_models)
rospack_add_library(PlanningRobotModels src/robot_models/kinematic.cpp)
