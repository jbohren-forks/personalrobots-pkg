cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(laser_scan)
rospack_add_library(laser_scan src/laser_scan.cc )
rospack_add_library(laser_median_filter src/median_filter.cpp )

rospack_add_executable(median_node src/median_filter_node.cpp)
target_link_libraries(median_node laser_median_filter)