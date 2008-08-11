cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(fake_localization)
rospack_add_executable(fake_localization fake_localization.cpp)
