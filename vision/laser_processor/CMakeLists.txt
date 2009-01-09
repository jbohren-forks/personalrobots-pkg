cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(laser_processor)
rospack_add_library(laser_processor laser_processor.cpp)