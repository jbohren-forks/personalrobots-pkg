cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(filter_demo)
genmsg()
gensrv()

rospack_add_library(filter_demo
                    src/filtering_example.cpp)

rospack_add_executable(filtering_example
                       src/filtering_example.cpp)

