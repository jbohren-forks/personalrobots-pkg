cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(laser_view)
rospack_add_executable(laser_view laser_view.cpp)
