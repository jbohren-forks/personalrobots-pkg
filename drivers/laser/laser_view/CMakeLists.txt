cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(laser_view)
rospack_add_executable(laser_view laser_view.cpp)
