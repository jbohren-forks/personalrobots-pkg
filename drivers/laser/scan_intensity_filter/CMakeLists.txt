cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(scan_intensity_filter)

rospack_add_executable(scan_intensity_filter_node scan_intensity_filter.cpp)

