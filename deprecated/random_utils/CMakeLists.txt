cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(random_utils)

rospack_add_boost_directories()

rospack_add_library(random_utils src/random_utils.cpp)
rospack_link_boost(random_utils thread)
rospack_add_executable(test_random_utils src/test.cpp)
target_link_libraries(test_random_utils random_utils)
