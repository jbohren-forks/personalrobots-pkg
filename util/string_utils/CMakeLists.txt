cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(string_utils)
rospack_add_library(string_utils src/string_utils.cpp)

rospack_add_gtest(test_string_utils test/test_string_utils.cpp)
target_link_libraries(test_string_utils string_utils)
