cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(math_utils)
rospack_add_library(math_utils src/MathExpression.cpp)
