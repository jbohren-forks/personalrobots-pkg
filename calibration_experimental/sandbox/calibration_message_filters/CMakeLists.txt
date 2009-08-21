cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(calibration_message_filters)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(calibration_message_filters src/stationary_checker.cpp
                                                src/joint_states_deflater.cpp)

add_subdirectory(test EXCLUDE_FROM_ALL)
