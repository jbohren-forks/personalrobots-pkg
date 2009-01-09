cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(joy_node)
rospack_add_executable(joy_node joy_node.cpp)

