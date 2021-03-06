cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rospack(pr2_calibration_actions)

genmsg()
gensrv()

rospack_add_library(${PROJECT_NAME} src/pixel_channel.cpp
                                    src/joint_states_channel.cpp
                                    src/robot_pixels_capture.cpp)

#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)


#rospack_add_executable(run_action_stereo_cb src/run_action_stereo_cb.cpp)
#target_link_libraries(run_action_stereo_cb ${PROJECT_NAME})

rospack_add_executable(test_stereo_cb src/test_stereo_cb.cpp)
#target_link_libraries(test_stereo_cb ${PROJECT_NAME})

rospack_add_executable(capture_hand_led src/capture_hand_led.cpp)

rospack_add_executable(run_robot_pixels_capture src/run_robot_pixels_capture.cpp)
target_link_libraries(run_robot_pixels_capture ${PROJECT_NAME})

rospack_add_executable(capture_robot_pixels_action_server src/capture_robot_pixels_action_server.cpp)
target_link_libraries(capture_robot_pixels_action_server ${PROJECT_NAME})

rospack_add_executable(capture_robot_pixels_sample_client src/capture_robot_pixels_sample_client.cpp)
target_link_libraries(capture_robot_pixels_sample_client ${PROJECT_NAME})

rospack_add_executable(capture_robot_pixels_client src/capture_robot_pixels_client.cpp)
target_link_libraries(capture_robot_pixels_client ${PROJECT_NAME})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(test EXCLUDE_FROM_ALL)
