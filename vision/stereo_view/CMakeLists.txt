cmake_minimum_required(VERSION 2.6)
include(rosbuild)

rospack(stereo_view)

rospack_add_executable(stereo_view stereo_view.cpp)
rospack_add_executable(stereo_view_pixel_info stereo_view_pixel_info.cpp)
