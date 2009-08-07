cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rospack(move_base)
genmsg()

# Library
rospack_add_library(move_base src/move_base_old.cpp)

# move_base
rospack_add_executable(bin/move_base src/move_base_old.cpp)

rospack_add_executable(bin/move_base_new src/move_base.cpp)
