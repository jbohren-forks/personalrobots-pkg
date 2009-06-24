cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(costmap_2d_tutorials)
genmsg()

rospack_add_executable(bin/costmap_test src/costmap_test.cpp)
