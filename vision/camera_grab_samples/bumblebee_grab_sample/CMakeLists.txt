cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(bumblebee_grab_sample)
rospack_add_executable(bumblebee_grab_sample bumblebee_grab_sample.cpp)
