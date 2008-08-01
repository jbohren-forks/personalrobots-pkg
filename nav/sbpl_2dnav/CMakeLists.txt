cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(sbpl_2dnav)

rospack_add_executable(bin/sbpl_2dnav
src/sbpl_2dnav.cpp)



