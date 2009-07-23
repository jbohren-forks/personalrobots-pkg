cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Release)

rospack_add_boost_directories()

rospack(test_collision_space)

rospack_add_executable(test_cs_point_inclusion src/test_cs_point_inclusion.cpp)

rospack_add_executable(collision_test_speed src/collision_test_speed.cpp)
rospack_link_boost(collision_test_speed thread)
