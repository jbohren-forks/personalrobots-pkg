cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Release)

rospack(dc1394_cam_server)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(dc1394_cam_server src/dc1394_cam_server/dc1394_cam_server.cpp)
rospack_add_executable(check_params src/check_params/check_params.cpp)
