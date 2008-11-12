cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(resource_locator)
rospack_add_library(resource_locator src/reslocator.cpp)
