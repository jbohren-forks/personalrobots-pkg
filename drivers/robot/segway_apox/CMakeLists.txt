cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(segway_apox)
rospack_add_executable(segway_apox segway.cpp serial.cpp usbcan.cpp)
