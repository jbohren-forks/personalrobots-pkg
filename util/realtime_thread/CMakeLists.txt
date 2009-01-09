cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(realtime_thread)
rospack_add_library(realtime_thread src/realtime_thread.cpp)
