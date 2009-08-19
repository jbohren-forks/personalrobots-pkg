cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(stereo_view)

rospack_add_library(flesh flesh/CHelpFleshDetect.cpp)
rospack_add_executable(bin/stereo_process stereo_process.cpp)
rospack_add_executable(bin/stereo_view stereo_view.cpp)
target_link_libraries(bin/stereo_process flesh)

