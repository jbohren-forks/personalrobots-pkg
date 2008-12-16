cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE Release)
rospack(cloud_io)
add_definitions(-Wall)

rospack_add_library(cloud_io src/read.cpp src/write.cpp src/misc.cpp)
