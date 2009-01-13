cmake_minimum_required(VERSION 2.4.6)

# Must have <linux/joystick.h>
include(CheckIncludeFiles)
check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)
if(NOT HAVE_LINUX_JOYSTICK_H)
  message(FATAL_ERROR "can't proceed without <linux/joystick.h>")
endif(NOT HAVE_LINUX_JOYSTICK_H)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(joy)
genmsg()
include_directories(msg/cpp)
rospack_add_executable(joy joy.cpp)

