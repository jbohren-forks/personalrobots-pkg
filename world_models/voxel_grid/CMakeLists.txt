cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(voxel_grid)
rospack_add_executable(voxel_grid src/voxel_grid.cpp)
