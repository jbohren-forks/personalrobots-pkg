cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(neven_face_detector)

set(ROS_BUILD_TYPE Release)

rospack_add_executable(nevenFD nevenFD.cpp)
