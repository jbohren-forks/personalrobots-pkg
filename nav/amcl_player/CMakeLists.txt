cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(amcl_player)
rospack_add_executable(amcl_player amcl_player.cc)
target_link_libraries(amcl_player playerdrivers)

rospack_add_executable(cli cli.cpp)

