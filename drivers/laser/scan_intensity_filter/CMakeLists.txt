cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(scan_intensity_filter)

rospack_add_executable(scan_intensity_filter_node scan_intensity_filter.cpp)

