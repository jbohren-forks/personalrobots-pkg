cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(amcl_player)
rospack_add_executable(amcl_player amcl_player.cc)
target_link_libraries(amcl_player playerdrivers)

