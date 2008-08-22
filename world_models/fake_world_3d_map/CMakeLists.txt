cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(fake_world_3d_map)
rospack_add_executable(fake_world_3d_map src/fake_world_3d_map.cpp)
