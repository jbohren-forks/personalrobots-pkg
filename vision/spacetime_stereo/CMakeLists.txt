cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(spacetime_stereo)
set(ROS_COMPILE_FLAGS "-Wall -g -O3 -fomit-frame-pointer -funroll-loops")
rospack_add_executable(sts sts.cpp)





