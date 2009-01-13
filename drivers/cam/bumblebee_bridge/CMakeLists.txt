cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(bumblebee_bridge)
rospack_add_executable(bumblebee_bridge bumblebee_bridge.cpp)
