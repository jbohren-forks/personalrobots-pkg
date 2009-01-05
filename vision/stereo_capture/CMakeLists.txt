cmake_minimum_required(VERSION 2.6)
include(rosbuild)

rospack(stereo_capture)

rospack_add_executable(stereo_capture src/stereo_capture.cpp)
rospack_add_executable(project_light src/project_light.cpp)
