cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rospack(nav)
genmsg()

rospack_add_boost_directories()


# Library
rospack_add_library(nav src/nav.cpp)
rospack_link_boost(nav thread)

# move_base
rospack_add_executable(bin/planner_test src/nav.cpp)
