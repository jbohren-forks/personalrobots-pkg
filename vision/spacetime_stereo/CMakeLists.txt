cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(spacetime_stereo)
set(ROS_COMPILE_FLAGS "-Wall -g -O3 -fomit-frame-pointer -funroll-loops")
rospack_add_executable(sts_node sts_node.cpp)
rospack_add_executable(sts_dynamic sts_dynamic.cpp)
#rospack_add_executable(sts sts.cpp)


