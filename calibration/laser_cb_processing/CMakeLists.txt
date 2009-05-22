cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(laser_cb_processing)

rospack_add_executable(laser_cb_processing src/laser_cb_processing.cpp)
rospack_add_executable(pr2_tilt_laser_projector src/pr2_tilt_laser_projector.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
