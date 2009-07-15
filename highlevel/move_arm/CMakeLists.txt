cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(move_arm)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# main executable 
rospack_add_executable(move_arm src/move_arm.cpp)
rospack_link_boost(move_arm thread)

# actuate gripper action
rospack_add_executable(actuate_gripper_action src/actuate_gripper_action.cpp)

# debugging tools

rospack_add_executable(arm_cmd_line src/arm_cmd_line.cpp)
rospack_link_boost(arm_cmd_line thread)


# simple tests
rospack_add_executable(oscillate_move test/oscillate_move.cpp)
rospack_link_boost(oscillate_move thread)

# actuate gripper action test
rospack_add_executable(actuate_gripper_action_test test/test_actuate_gripper.cpp)
