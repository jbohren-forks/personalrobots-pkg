cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(laser_scan_annotator)
genmsg()

rospack_add_executable(laser_scan_annotator_node src/laser_scan_annotator_node.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)