cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(voxel_grid)
rospack_add_executable(voxel_grid src/voxel_grid.cpp)
