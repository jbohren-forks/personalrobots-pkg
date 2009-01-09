cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rospack(sbpl_2dnav)

rospack_add_executable(bin/sbpl_2dnav
src/sbpl_2dnav.cpp)



