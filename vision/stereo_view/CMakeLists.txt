cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(stereo_view)

rospack_add_executable(stereo_view stereo_view.cpp)
rospack_add_executable(stereo_view_pixel_info stereo_view_pixel_info.cpp)
