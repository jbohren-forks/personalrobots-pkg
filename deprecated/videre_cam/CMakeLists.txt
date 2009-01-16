cmake_minimum_required(VERSION 2.6)
include(rosbuild)

set(ROS_BUILD_TYPE Release)

rospack(videre_cam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(viderecam src/videre_cam/videre_cam.cpp)

