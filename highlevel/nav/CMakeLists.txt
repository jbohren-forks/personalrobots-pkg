cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rospack(nav)
genmsg()

rospack_add_boost_directories()


# Library
rospack_add_library(nav src/move_base_action.cpp)
rospack_link_boost(nav thread)

# move_base
rospack_add_executable(bin/move_base src/move_base_action.cpp)

#local controller
rospack_add_executable(bin/local_controller src/local_controller_action.cpp)
