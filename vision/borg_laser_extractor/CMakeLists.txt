cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(borg_laser_extractor)
add_definitions(-Wall -Werror)
rospack_add_executable(laser_extractor laser_extractor.cpp)

