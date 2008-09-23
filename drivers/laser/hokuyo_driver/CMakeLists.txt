cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(hokuyo_driver)
rospack_add_library(hokuyo hokuyo.cpp)
