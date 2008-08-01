cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(calib_converter)
add_definitions(-Wall)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
rospack_add_executable(calib_converter src/calib_converter.cpp)


