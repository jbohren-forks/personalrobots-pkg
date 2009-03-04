cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(laser_scan)
genmsg()

rospack_add_boost_directories()

rospack_add_library(laser_scan src/laser_scan.cc )
rospack_link_boost(laser_scan thread)

rospack_add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)

rospack_add_executable(scan_shadows_filter_node src/scan_shadows_filter.cpp)
target_link_libraries (scan_shadows_filter_node laser_scan)

