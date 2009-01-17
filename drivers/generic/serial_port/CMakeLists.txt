cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(serial_port)
rospack_add_library(serial_port src/libserial_port/lightweightserial.cpp)

