cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(pr2_calibration_actions)

genmsg()

rospack_add_library(${PROJECT_NAME} src/stereo_cb_action.cpp)
rospack_add_boost_directories()
rospack_link_boost(${PROJECT_NAME} thread)


rospack_add_executable(run_action_stereo_cb src/run_action_stereo_cb.cpp)
target_link_libraries(run_action_stereo_cb ${PROJECT_NAME})

rospack_add_executable(test_stereo_cb src/test_stereo_cb.cpp)
target_link_libraries(test_stereo_cb ${PROJECT_NAME})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)