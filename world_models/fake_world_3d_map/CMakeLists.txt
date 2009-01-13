cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(fake_world_3d_map)
rospack_add_executable(fake_world_3d_map src/fake_world_3d_map.cpp)
