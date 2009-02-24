cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_collision_space)

rospack_add_executable(test_cs_util src/test_cs_util.cpp)
