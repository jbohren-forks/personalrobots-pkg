cmake_minimum_required(VERSION 2.6)
include(rosbuild)

set(ROS_BUILD_TYPE RelWithDebInfo)

rospack(dcam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(src)

