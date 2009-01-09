cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(pr2_etherCAT)
rospack_add_executable(pr2_etherCAT src/main.cpp)
