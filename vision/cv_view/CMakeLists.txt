cmake_minimum_required(VERSION 2.6)
include(rosbuild)

rospack(cv_view)
genmsg()

rospack_add_executable(cv_view src/cv_view.cpp)
rospack_add_executable(cv_view_array src/cv_view_array.cpp)
rospack_add_executable(cv_view_array_overlay src/cv_view_array_overlay.cpp)
