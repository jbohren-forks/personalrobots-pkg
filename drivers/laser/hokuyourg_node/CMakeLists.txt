cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(hokuyourg_node)
rospack_add_executable(hokuyourg_node hokuyourg_player.cc)
