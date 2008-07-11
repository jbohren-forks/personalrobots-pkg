cmake_minimum_required(VERSION 2.6)

# Must have <linux/joystick.h>
include(CheckIncludeFiles)
check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)
if(NOT HAVE_LINUX_JOYSTICK_H)
  message(FATAL_ERROR "can't proceed without <linux/joystick.h>")
endif(NOT HAVE_LINUX_JOYSTICK_H)

include(rosbuild)
rospack(joy)
genmsg()
rospack_add_executable(joy joy.cpp)

