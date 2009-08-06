cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rospack(move_base)
genmsg()

rospack_add_boost_directories()


# Library
rospack_add_library(move_base src/move_base_old.cpp)
rospack_link_boost(move_base thread)
rospack_link_boost(move_base filesystem)
target_link_libraries (move_base PocoUtil)

# move_base
rospack_add_executable(bin/move_base src/move_base_old.cpp)
rospack_link_boost(bin/move_base thread)
rospack_link_boost(bin/move_base filesystem)
target_link_libraries (bin/move_base PocoUtil)

rospack_add_executable(bin/move_base_new src/move_base.cpp)
rospack_link_boost(bin/move_base_new thread)
rospack_link_boost(bin/move_base_new filesystem)
target_link_libraries (bin/move_base_new PocoUtil)
