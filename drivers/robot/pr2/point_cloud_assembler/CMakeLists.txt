cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(point_cloud_assembler)
#genmsg()
#find_library(curl REQUIRED)

rospack_add_executable(point_cloud_assembler src/point_cloud_assembler.cpp)

