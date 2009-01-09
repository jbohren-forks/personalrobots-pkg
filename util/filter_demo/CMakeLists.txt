cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(filter_demo)
genmsg()
gensrv()

rospack_add_library(filter_demo
                    src/filtering_example.cpp)

rospack_add_executable(filtering_example
                       src/filtering_example.cpp)

