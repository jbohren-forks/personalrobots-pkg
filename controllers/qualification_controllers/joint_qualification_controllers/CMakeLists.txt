cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_STATIC_LIBS true)
set(ROS_BUILD_SHARED_LIBS false)
rospack(joint_qualification_controllers)
genmsg()
gensrv()
rospack_add_library(joint_qualification_controllers 
src/hysteresis_controller.cpp
src/sine_sweep_controller.cpp
src/backlash_controller.cpp)
