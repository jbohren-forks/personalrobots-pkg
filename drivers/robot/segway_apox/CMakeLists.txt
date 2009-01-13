cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(segway_apox)
rospack_add_executable(segway_apox segway.cpp serial.cpp usbcan.cpp)
