cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(map_saver)
rospack_add_executable(map_saver src/map_saver.cpp)
