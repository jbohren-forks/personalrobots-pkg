cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(teleop_base_keyboard)
rospack_add_executable(teleop_base_keyboard teleop_base_keyboard.cc)
