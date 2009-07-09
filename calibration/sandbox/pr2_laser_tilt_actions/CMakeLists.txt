cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(pr2_laser_tilt_actions)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

genmsg()

rospack_add_library(${PROJECT_NAME} src/single_scan_action.cpp)
rospack_add_boost_directories()
rospack_link_boost(${PROJECT_NAME} thread)

rospack_add_executable(run_action_single_scan src/run_action_single_scan.cpp)
target_link_libraries(run_action_single_scan ${PROJECT_NAME})

rospack_add_executable(test_single_scan src/test_single_scan.cpp)
target_link_libraries(test_single_scan ${PROJECT_NAME})

#rospack_add_executable(test_spin src/test_spin.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
