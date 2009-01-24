cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(mechanism_control)

rospack_add_boost_directories()

rospack_add_library(mechanism_control src/mechanism_control.cpp)
rospack_link_boost(mechanism_control thread)
#rospack_add_executable(ms_publisher_test test/ms_publisher_test.cpp)
#rospack_add_rostest(test/test-mechanism-state-cpp.xml)
#rospack_add_rostest(test/test-mechanism-state-py.xml)
