cmake_minimum_required(VERSION 2.6)
include(rosbuild)
set(ROS_BUILD_TYPE Release)
rospack(dcam_node)

rospack_add_executable(dcam_node dcam_node.cpp)
