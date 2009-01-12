cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(laser_scan)
rospack_add_library(laser_scan src/laser_scan.cc )
rospack_add_library(laser_median_filter src/median_filter.cpp )

rospack_add_executable(median_node src/median_filter_node.cpp)
target_link_libraries(median_node laser_median_filter)

rospack_add_executable(scan_shadows_filter_node src/scan_shadows_filter.cpp)
target_link_libraries (scan_shadows_filter_node laser_scan)

rospack_add_executable(scan_intensity_filter src/scan_intensity_filter.cpp)
target_link_libraries (scan_intensity_filter laser_scan)
