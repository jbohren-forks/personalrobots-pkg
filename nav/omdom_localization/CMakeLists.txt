cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(odom_localization)
rospack_add_executable(odom_localization odom_localization.cpp)
