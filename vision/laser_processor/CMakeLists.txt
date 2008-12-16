cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(laser_processor)
rospack_add_library(laser_processor laser_processor.cpp)