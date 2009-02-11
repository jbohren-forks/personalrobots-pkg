cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(mechanism_controller_test)

rospack_add_boost_directories()

rospack_add_executable(bin/mechanism_controller_test src/mechanism_controller_test.cpp src/null_hardware.cpp)
