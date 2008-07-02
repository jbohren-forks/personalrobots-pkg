cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(pr_robot_desc)
rospack_add_library(libURDF src/urdf/URDF.cpp src/urdf/MathExpression.cpp)
