cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)
rospack(teleop_arm_keyboard)

rospack_add_executable(teleop_arm_keyboard teleop_arm_keyboard.cc)


