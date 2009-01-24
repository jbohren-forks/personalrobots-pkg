cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(laser_scan_annotator)
genmsg()

rospack_add_boost_directories()

rospack_add_executable(laser_scan_annotator_node src/laser_scan_annotator_node.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
