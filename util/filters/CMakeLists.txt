cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(filters)
rospack_add_library(median_filter src/median.cpp )