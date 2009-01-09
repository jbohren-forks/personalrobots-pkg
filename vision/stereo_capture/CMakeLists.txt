cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(stereo_capture)

rospack_add_executable(stereo_capture src/stereo_capture.cpp)
rospack_add_executable(project_light src/project_light.cpp)
