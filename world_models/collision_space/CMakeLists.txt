cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(collision_space)

rospack_add_boost_directories()

set(CMAKE_BUILD_TYPE Release)

rospack_add_library(collision_space src/collision_space/output.cpp
                                    src/collision_space/point_inclusion.cpp
				    src/collision_space/environment.cpp
				    src/collision_space/environmentODE.cpp)	
rospack_link_boost(collision_space thread)


# Unit tests
rospack_add_gtest(test_point_inclusion test/test_point_inclusion.cpp)
rospack_link_boost(test_point_inclusion thread)
target_link_libraries(test_point_inclusion collision_space)
