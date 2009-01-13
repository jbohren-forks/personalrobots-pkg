cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(teleop_base_keyboard)
rospack_add_executable(teleop_base_keyboard teleop_base_keyboard.cc)
