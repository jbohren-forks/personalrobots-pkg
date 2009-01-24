cmake_minimum_required(VERSION 2.4.6)
set(ROS_BUILD_TYPE Release)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(world_3d_map)

rospack_add_boost_directories()

rospack_add_executable(world_3d_map src/world_3d_map.cpp)
