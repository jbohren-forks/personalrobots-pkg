cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(costmap_2d)
rospack_add_library(costmap_2d src/costmap_2d.cpp)
