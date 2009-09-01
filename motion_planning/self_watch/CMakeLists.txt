cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(self_watch)
rospack_add_executable(self_watch self_watch.cpp)
