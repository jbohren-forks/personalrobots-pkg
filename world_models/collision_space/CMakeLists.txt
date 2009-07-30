cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(collision_space)

set(ROS_BUILD_TYPE Release)

rospack_add_boost_directories()

rospack_add_library(collision_space src/environment_objects.cpp
				    src/environment.cpp
				    src/environmentBullet.cpp
				    src/environmentODE.cpp)

rospack_link_boost(collision_space thread)
