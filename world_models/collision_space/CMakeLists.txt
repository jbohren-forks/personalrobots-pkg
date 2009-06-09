cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(collision_space)

rospack_add_boost_directories()

set(ROS_BUILD_TYPE Debug)

rospack_add_library(collision_space src/bodies.cpp
				    src/environment.cpp
				    src/environmentODESimple.cpp
				    src/environmentODE.cpp)	
rospack_link_boost(collision_space thread)


# Unit tests
rospack_add_gtest(test_point_inclusion test/test_point_inclusion.cpp)
rospack_link_boost(test_point_inclusion thread)
target_link_libraries(test_point_inclusion collision_space)
