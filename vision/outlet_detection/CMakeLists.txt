cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(outlet_detection)
add_definitions(-Wall -O3 -DNDEBUG)
#add_definitions(-Wall -O0 -g)

rospack_add_executable(bin/orange_detect src/orange_detect.cpp)
