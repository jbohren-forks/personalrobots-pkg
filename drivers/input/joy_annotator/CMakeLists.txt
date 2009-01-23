cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(joy_annotator)
rospack_add_executable(bin/joy_annotator joy_annotator.cpp)

