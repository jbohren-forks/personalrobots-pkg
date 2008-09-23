cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(hokuyo_utm_driver)
rospack_add_library(hokuyoutm hokuyo_utm.cpp)
