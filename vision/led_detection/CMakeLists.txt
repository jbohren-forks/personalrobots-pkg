cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(led_detection)

rospack_add_library(led_detection src/led_detector.cpp )

rospack_add_executable(led_detector_node src/led_detector_node.cpp)
target_link_libraries(led_detector_node led_detection)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)