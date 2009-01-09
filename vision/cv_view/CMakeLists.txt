cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(cv_view)
genmsg()

rospack_add_executable(cv_view src/cv_view.cpp)
rospack_add_executable(cv_view_minimal minimal/cv_view.cpp)
rospack_add_executable(cv_view_array src/cv_view_array.cpp)
rospack_add_executable(cv_view_dcam dcam/cv_view.cpp)
rospack_add_executable(cv_view_array_overlay src/cv_view_array_overlay.cpp)