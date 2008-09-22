cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(hokuyourg_driver)
rospack_add_library(urg urg_laser.cc)
