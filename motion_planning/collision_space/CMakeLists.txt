cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(collision_space)
rospack_add_library(collision_space src/collision_space/environment.cpp
#				    src/collision_space/environmentOctree.cpp
				    src/collision_space/environmentODE.cpp)	

# Unit tests
rospack_add_gtest_future(test_util test/test_util.cpp)
target_link_libraries(test_util collision_space)
