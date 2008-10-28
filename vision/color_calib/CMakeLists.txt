cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(color_calib)
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(calib_test calib_test.cpp)
