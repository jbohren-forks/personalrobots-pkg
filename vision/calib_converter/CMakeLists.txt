cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(calib_converter)

rospack_add_boost_directories()

add_definitions(-Wall)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
rospack_add_executable(calib_converter src/calib_converter.cpp)
rospack_link_boost(calib_converter thread)

