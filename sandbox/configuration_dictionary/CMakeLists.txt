cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(configuration_dictionary)

rospack_add_library(configuration_dictionary configuration_dictionary.cpp)
#rospack_add_executable(t configuration_dictionary.cpp test.cpp)
