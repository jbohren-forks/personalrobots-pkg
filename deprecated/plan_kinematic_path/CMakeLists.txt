cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(plan_kinematic_path)
rospack_add_executable(plan_kinematic_path src/plan_kinematic_path.cpp)
