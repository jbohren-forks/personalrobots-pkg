cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(collision_space)

rospack_add_boost_directories()

rospack_add_library(collision_space src/collision_space/environment.cpp
#				    src/collision_space/environmentOctree.cpp
				    src/collision_space/environmentODE.cpp)	
rospack_link_boost(collision_space thread)


# Unit tests
rospack_add_gtest(test_util test/test_util.cpp)
target_link_libraries(test_util collision_space)
